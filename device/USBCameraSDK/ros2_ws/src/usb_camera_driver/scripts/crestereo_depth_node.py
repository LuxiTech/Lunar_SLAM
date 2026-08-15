#!/usr/bin/env python3
"""CREStereo ONNX RGB-D frontend for the calibrated USB stereo camera.

The node intentionally lives beside, rather than inside, the production VPI
frontend.  It keeps only the newest synchronized stereo pair while inference
is busy so RTAB-Map receives fresh frames instead of an ever-growing backlog.
"""

from __future__ import annotations

from array import array
from concurrent.futures import ThreadPoolExecutor
import gc
from pathlib import Path
import threading
import time
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import RGBDImage
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2, PointField
from std_msgs.msg import Header


def _read_matrix(storage: cv2.FileStorage, name: str, shape: Tuple[int, int]) -> np.ndarray:
    matrix = storage.getNode(name).mat()
    if matrix is None or matrix.shape != shape:
        raise RuntimeError(f"calibration matrix {name!r} must be {shape[0]}x{shape[1]}")
    return np.asarray(matrix, dtype=np.float64)


def _scale_projection(projection: np.ndarray, sx: float, sy: float) -> np.ndarray:
    result = projection.copy()
    result[0, :] *= sx
    result[1, :] *= sy
    return result


def _image_message(header: Header, encoding: str, image: np.ndarray) -> Image:
    contiguous = np.ascontiguousarray(image)
    message = Image()
    message.header = header
    message.height = contiguous.shape[0]
    message.width = contiguous.shape[1]
    message.encoding = encoding
    message.is_bigendian = False
    message.step = contiguous.strides[0]
    # rclpy's generated setter validates a plain bytes object one element at
    # a time when Python assertions are enabled (~90 ms for this RGB-D pair on
    # the NX).  array('B') is the message's native representation and takes
    # the generated fast path without changing or re-encoding any pixels.
    message.data = array("B", contiguous.tobytes())
    return message


def _camera_info(header: Header, size: Tuple[int, int], projection: np.ndarray) -> CameraInfo:
    width, height = size
    message = CameraInfo()
    message.header = header
    message.width = width
    message.height = height
    message.distortion_model = "plumb_bob"
    message.d = [0.0] * 5
    message.k = [
        float(projection[0, 0]), 0.0, float(projection[0, 2]),
        0.0, float(projection[1, 1]), float(projection[1, 2]),
        0.0, 0.0, 1.0,
    ]
    message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    message.p = [float(value) for value in projection.reshape(-1)]
    return message


def _apply_ground_plane_prior(
    depth_mm: np.ndarray,
    projection: np.ndarray,
    camera_height_m: float,
    below_ground_tolerance_m: float,
    minimum_row_ratio: float,
    minimum_depth_m: float,
    maximum_depth_m: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Project physically impossible below-ground returns onto the floor.

    For a level, forward-facing optical frame, ``y = height`` is the ground
    plane.  A valid stereo return whose projected optical Y is appreciably
    larger than the measured camera height would lie below that plane.  It is
    therefore a bad low-texture floor match, not a nearer obstacle.  Moving
    only those returns to the ray/ground intersection restores the floor while
    preserving every closer obstacle and every unknown (zero-depth) pixel.

    The mask is returned so the published disparity can remain consistent
    with the corrected metric depth.
    """
    corrected_mask = np.zeros(depth_mm.shape, dtype=bool)
    if camera_height_m <= 0.0 or depth_mm.size == 0:
        return depth_mm, corrected_mask

    height, _ = depth_mm.shape
    fy = float(projection[1, 1])
    cy = float(projection[1, 2])
    if not np.isfinite(fy) or fy <= 0.0 or not np.isfinite(cy):
        raise ValueError("ground-plane prior requires finite positive camera intrinsics")

    rows = np.arange(height, dtype=np.float32)[:, None]
    downward_ratio = (rows - cy) / fy
    minimum_row = max(cy + 1.0, minimum_row_ratio * height)
    depth_m = depth_mm.astype(np.float32) * 0.001
    corrected_mask = (
        (depth_mm > 0)
        & (rows >= minimum_row)
        & (downward_ratio > 0.0)
        & (
            depth_m * downward_ratio
            > camera_height_m + below_ground_tolerance_m
        )
    )
    if not np.any(corrected_mask):
        return depth_mm, corrected_mask

    expected_depth = np.divide(
        camera_height_m,
        downward_ratio,
        out=np.zeros_like(downward_ratio),
        where=downward_ratio > 0.0,
    )
    corrected_mask &= (
        (expected_depth >= minimum_depth_m)
        & (expected_depth <= maximum_depth_m)
    )
    if not np.any(corrected_mask):
        return depth_mm, corrected_mask

    result = depth_mm.copy()
    expected_mm = np.clip(
        np.rint(expected_depth * 1000.0), 0.0, 65535.0
    ).astype(np.uint16)
    # expected_depth is one value per row and broadcasts across columns.
    result[corrected_mask] = np.broadcast_to(
        expected_mm, depth_mm.shape
    )[corrected_mask]
    return result, corrected_mask


def _normalize_stereo_photometry(
    left_bgr: np.ndarray,
    right_bgr: np.ndarray,
    valid_mask: np.ndarray,
    minimum_gain: float,
    maximum_gain: float,
    maximum_offset: float,
) -> Tuple[np.ndarray, float, float]:
    """Match the right sensor's robust intensity range to the left sensor.

    The two UVC sensors share one manual exposure/gain command, but their
    analogue responses are not identical. CREStereo expects the same scene
    radiance in both inputs; an independent offset makes specular floor
    patches look like valid but different objects. Estimate one affine
    transform from common rectified pixels and apply it only to the model's
    right input. The published left RGB image is deliberately untouched.
    """
    left = np.asarray(left_bgr)
    right = np.asarray(right_bgr)
    mask = np.asarray(valid_mask, dtype=bool)
    if left.shape != right.shape or left.ndim != 3 or left.shape[2] != 3:
        raise ValueError("stereo photometric normalization requires matching BGR images")
    if mask.shape != left.shape[:2]:
        raise ValueError("stereo photometric mask must match the image geometry")
    if not 0.0 < minimum_gain <= maximum_gain or maximum_offset < 0.0:
        raise ValueError("invalid stereo photometric normalization limits")

    left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    # A 4x lattice is enough for robust percentiles and keeps this CPU stage
    # below the inference noise floor on the NX.
    samples = mask[::4, ::4]
    left_samples = left_gray[::4, ::4][samples]
    right_samples = right_gray[::4, ::4][samples]
    nonsaturated = (
        (left_samples > 4) & (left_samples < 251)
        & (right_samples > 4) & (right_samples < 251)
    )
    left_samples = left_samples[nonsaturated]
    right_samples = right_samples[nonsaturated]
    if left_samples.size < 128:
        return right.copy(), 1.0, 0.0

    left_low, left_middle, left_high = np.percentile(
        left_samples, (10.0, 50.0, 90.0)
    )
    right_low, right_middle, right_high = np.percentile(
        right_samples, (10.0, 50.0, 90.0)
    )
    gain = float(np.clip(
        (left_high - left_low) / max(right_high - right_low, 1.0),
        minimum_gain,
        maximum_gain,
    ))
    offset = float(np.clip(
        left_middle - gain * right_middle,
        -maximum_offset,
        maximum_offset,
    ))
    normalized = cv2.addWeighted(right, gain, right, 0.0, offset)
    return normalized, gain, offset


def _photometric_consistency_mask(
    left_bgr: np.ndarray,
    right_bgr: np.ndarray,
    disparity: np.ndarray,
    valid: np.ndarray,
    pixel_threshold: float,
    patch_size: int,
    patch_threshold: float,
) -> Tuple[np.ndarray, np.ndarray, float]:
    """Reject view-dependent and occluded matches after a subpixel warp.

    A single nearest-neighbour pixel test admits shiny-floor coincidences and
    unnecessarily rejects correct subpixel matches. Bilinear warping followed
    by a small mean-residual gate requires support from a real image patch,
    which is substantially harder for a specular highlight to fake.
    """
    left = np.asarray(left_bgr)
    right = np.asarray(right_bgr)
    flow = np.asarray(disparity, dtype=np.float32)
    mask = np.asarray(valid, dtype=bool).copy()
    if left.shape != right.shape or left.shape[:2] != flow.shape:
        raise ValueError("photometric consistency inputs must have matching geometry")
    if mask.shape != flow.shape:
        raise ValueError("photometric consistency mask must match disparity")
    if pixel_threshold <= 0.0:
        return mask, np.zeros(flow.shape, dtype=np.float32), 0.0
    if patch_size < 1 or patch_size % 2 == 0 or patch_threshold < 0.0:
        raise ValueError("invalid photometric patch configuration")

    height, width = flow.shape
    columns = np.arange(width, dtype=np.float32)[None, :]
    rows = np.broadcast_to(
        np.arange(height, dtype=np.float32)[:, None], flow.shape
    )
    right_columns = columns - flow
    mask &= (right_columns >= 0.0) & (right_columns <= width - 1.001)
    left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY).astype(np.float32)
    right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY).astype(np.float32)
    warped_right = cv2.remap(
        right_gray,
        right_columns,
        rows,
        cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )
    residual = left_gray - warped_right
    brightness_offset = float(np.median(residual[mask])) if np.any(mask) else 0.0
    difference = np.abs(residual - brightness_offset)
    mask &= difference <= pixel_threshold
    if patch_size > 1 and patch_threshold > 0.0:
        patch_difference = cv2.boxFilter(
            difference,
            cv2.CV_32F,
            (patch_size, patch_size),
            normalize=True,
            borderType=cv2.BORDER_REPLICATE,
        )
        mask &= patch_difference <= patch_threshold
    return mask, difference, brightness_offset


def _left_right_consistency_mask(
    left_disparity: np.ndarray,
    flipped_right_disparity: np.ndarray,
    valid: np.ndarray,
    maximum_difference: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Validate left depth with an independently inferred right disparity.

    CREStereo is trained for positive left-to-right disparity. To obtain the
    reverse field without changing that assumption, inference receives the
    horizontally flipped ``(right, left)`` pair. Flipping its output back
    yields a positive right-to-left disparity in the original right image.
    A real surface must predict the same displacement in both directions;
    reflections, occlusions and repeated floor texture usually do not.
    """
    left = np.asarray(left_disparity, dtype=np.float32)
    reverse = np.flip(
        np.asarray(flipped_right_disparity, dtype=np.float32), axis=1
    ).copy()
    mask = np.asarray(valid, dtype=bool).copy()
    if left.ndim != 2 or reverse.shape != left.shape or mask.shape != left.shape:
        raise ValueError("left-right consistency inputs must have matching geometry")
    if maximum_difference <= 0.0:
        return mask, np.zeros(left.shape, dtype=np.float32)

    height, width = left.shape
    columns = np.arange(width, dtype=np.float32)[None, :]
    rows = np.broadcast_to(
        np.arange(height, dtype=np.float32)[:, None], left.shape
    )
    right_columns = columns - left
    mask &= (right_columns >= 0.0) & (right_columns <= width - 1.001)
    sampled_reverse = cv2.remap(
        reverse,
        right_columns,
        rows,
        cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=-1.0,
    )
    difference = np.abs(left - sampled_reverse)
    mask &= (
        np.isfinite(sampled_reverse)
        & (sampled_reverse > 0.25)
        & (difference <= maximum_difference)
    )
    return mask, difference


def _edge_aware_upsample_disparity(
    disparity: np.ndarray,
    valid: np.ndarray,
    guide_bgr: np.ndarray,
    output_size: Tuple[int, int],
    disparity_scale: float,
    radius: int,
    epsilon: float,
) -> np.ndarray:
    """Upsample low-resolution disparity without copying its block grid.

    Disparity, rather than metric depth, is interpolated so planar surfaces
    stay planar. A high-resolution rectified RGB guide then keeps the smoothed
    disparity discontinuities on visible object boundaries. Invalid model
    pixels remain invalid; this does not invent depth in CREStereo holes.
    """
    source = np.asarray(disparity, dtype=np.float32)
    mask = np.asarray(valid, dtype=bool)
    guide = np.asarray(guide_bgr)
    width, height = output_size
    if source.ndim != 2 or mask.shape != source.shape:
        raise ValueError("disparity and validity mask must be matching 2-D arrays")
    if guide.shape[:2] != (height, width) or guide.ndim != 3:
        raise ValueError("edge-aware disparity guide must match output size")
    if width <= 0 or height <= 0 or disparity_scale <= 0.0:
        raise ValueError("invalid disparity output geometry")
    if radius < 1 or epsilon <= 0.0:
        raise ValueError("guided disparity parameters must be positive")
    if not hasattr(cv2, "ximgproc") or not hasattr(cv2.ximgproc, "guidedFilter"):
        raise RuntimeError("OpenCV ximgproc guidedFilter is required")

    weights = mask.astype(np.float32)
    weighted = np.where(mask, source, 0.0)
    weighted = cv2.resize(weighted, output_size, interpolation=cv2.INTER_LINEAR)
    weights = cv2.resize(weights, output_size, interpolation=cv2.INTER_LINEAR)
    upsampled = np.divide(
        weighted,
        np.maximum(weights, 0.05),
        out=np.zeros_like(weighted),
    ) * disparity_scale
    guide_gray = cv2.cvtColor(guide, cv2.COLOR_BGR2GRAY)
    upsampled = cv2.ximgproc.guidedFilter(
        guide_gray, upsampled, radius, epsilon
    )
    output_valid = cv2.resize(
        mask.astype(np.uint8), output_size, interpolation=cv2.INTER_NEAREST
    ).astype(bool)
    output_valid &= np.isfinite(upsampled) & (upsampled > 0.0)
    upsampled[~output_valid] = 0.0
    return upsampled.astype(np.float32, copy=False)


def _point_cloud_message(
    header: Header,
    color: np.ndarray,
    depth_mm: np.ndarray,
    projection: np.ndarray,
    maximum_depth_m: float,
    far_sparse_start_m: float,
    far_sparse_pixel_step: int,
    medium_maximum_depth_m: float = 0.0,
    medium_sparse_pixel_step: int = 1,
) -> PointCloud2:
    depth_m = depth_mm.astype(np.float32) * 0.001
    valid = depth_mm > 0
    if maximum_depth_m > 0.0:
        valid &= depth_m <= maximum_depth_m
    if far_sparse_start_m > 0.0 and far_sparse_pixel_step > 1:
        # Keep the 4 m boundary dense.  A finer 4-6 m lattice preserves room
        # shape, while a coarser 6-10 m lattice bounds map and DDS load.  Slice
        # assignment avoids full-frame int64 coordinate grids on the NX.
        far_lattice = np.zeros(depth_mm.shape, dtype=bool)
        far_lattice[::far_sparse_pixel_step, ::far_sparse_pixel_step] = True
        if (
            medium_maximum_depth_m > far_sparse_start_m
            and medium_sparse_pixel_step > 1
        ):
            medium_lattice = np.zeros(depth_mm.shape, dtype=bool)
            medium_lattice[
                ::medium_sparse_pixel_step, ::medium_sparse_pixel_step
            ] = True
            keep_tier = (
                (depth_m <= far_sparse_start_m)
                | (
                    (depth_m > far_sparse_start_m)
                    & (depth_m <= medium_maximum_depth_m)
                    & medium_lattice
                )
                | ((depth_m > medium_maximum_depth_m) & far_lattice)
            )
        else:
            keep_tier = (depth_m <= far_sparse_start_m) | far_lattice
        valid &= keep_tier
    rows, columns = np.nonzero(valid)
    z = depth_m[rows, columns]
    fx, fy = float(projection[0, 0]), float(projection[1, 1])
    cx, cy = float(projection[0, 2]), float(projection[1, 2])
    points = np.empty(
        z.size,
        dtype=np.dtype({
            "names": ["x", "y", "z", "rgb"],
            "formats": ["<f4", "<f4", "<f4", "<f4"],
            "offsets": [0, 4, 8, 12],
            "itemsize": 16,
        }),
    )
    points["x"] = (columns.astype(np.float32) - cx) * z / fx
    points["y"] = (rows.astype(np.float32) - cy) * z / fy
    points["z"] = z
    bgr = color[rows, columns].astype(np.uint32)
    packed_rgb = (bgr[:, 2] << 16) | (bgr[:, 1] << 8) | bgr[:, 0]
    points["rgb"] = packed_rgb.view(np.float32)

    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = int(z.size)
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 16
    cloud.row_step = cloud.width * cloud.point_step
    cloud.data = array("B", points.tobytes())
    cloud.is_dense = True
    return cloud


class CREStereoDepthNode(Node):
    def __init__(self) -> None:
        super().__init__("usb_crestereo_depth_node")
        calibration_file = self.declare_parameter("calibration_file", "").value
        model_path = str(Path(self.declare_parameter("model_path", "").value).expanduser())
        provider = self.declare_parameter("execution_provider", "tensorrt").value.lower()
        self.enable_cuda_graph = bool(self.declare_parameter(
            "enable_cuda_graph", True
        ).value)
        self.use_cuda_io_binding = bool(self.declare_parameter(
            "use_cuda_io_binding", True
        ).value)
        cache_path = str(Path(self.declare_parameter(
            "tensorrt_cache_path", "~/.cache/luxi/crestereo"
        ).value).expanduser())
        self.output_width = int(self.declare_parameter("output_width", 480).value)
        self.output_height = int(self.declare_parameter("output_height", 270).value)
        self.output_frame = self.declare_parameter(
            "output_frame_id", "left_camera_optical_frame"
        ).value
        self.compressed_input = bool(self.declare_parameter("compressed_input", True).value)
        self.minimum_depth = float(self.declare_parameter("min_depth_m", 0.4).value)
        self.maximum_depth = float(self.declare_parameter("max_depth_m", 10.0).value)
        self.maximum_vertical_flow = float(self.declare_parameter(
            "max_vertical_flow_px", 2.0
        ).value)
        self.photometric_threshold = float(self.declare_parameter(
            "photometric_threshold", 0.0
        ).value)
        self.photometric_patch_size = max(1, int(self.declare_parameter(
            "photometric_patch_size", 1
        ).value) | 1)
        self.photometric_patch_threshold = float(self.declare_parameter(
            "photometric_patch_threshold", 0.0
        ).value)
        self.left_right_consistency = bool(self.declare_parameter(
            "left_right_consistency", False
        ).value)
        self.left_right_maximum_difference = float(self.declare_parameter(
            "left_right_maximum_difference_px", 1.5
        ).value)
        left_right_model_value = self.declare_parameter(
            "left_right_model_path", ""
        ).value
        self.left_right_model_path = (
            str(Path(left_right_model_value).expanduser())
            if left_right_model_value else ""
        )
        self.stereo_photometric_normalization = bool(self.declare_parameter(
            "stereo_photometric_normalization", False
        ).value)
        self.stereo_photometric_minimum_gain = float(self.declare_parameter(
            "stereo_photometric_minimum_gain", 0.80
        ).value)
        self.stereo_photometric_maximum_gain = float(self.declare_parameter(
            "stereo_photometric_maximum_gain", 1.25
        ).value)
        self.stereo_photometric_maximum_offset = float(self.declare_parameter(
            "stereo_photometric_maximum_offset", 30.0
        ).value)
        self.far_artifact_depth = float(self.declare_parameter(
            "far_artifact_depth_m", 4.0
        ).value)
        self.far_support_size = max(1, int(self.declare_parameter(
            "far_artifact_support_size", 3
        ).value) | 1)
        self.far_minimum_support = max(1, int(self.declare_parameter(
            "far_artifact_minimum_support", 3
        ).value))
        self.median_size = max(1, int(self.declare_parameter(
            "output_median_filter_size", 3
        ).value) | 1)
        self.median_max_difference_m = float(self.declare_parameter(
            "output_median_max_difference_m", 0.12
        ).value)
        self.guided_upsampling_radius = max(0, int(self.declare_parameter(
            "guided_upsampling_radius", 0
        ).value))
        self.guided_upsampling_epsilon = float(self.declare_parameter(
            "guided_upsampling_epsilon", 100.0
        ).value)
        self.guided_upsampling_width = max(0, int(self.declare_parameter(
            "guided_upsampling_width", 0
        ).value))
        self.ground_plane_prior_enabled = bool(self.declare_parameter(
            "ground_plane_prior_enabled", False
        ).value)
        self.ground_plane_camera_height = float(self.declare_parameter(
            "ground_plane_camera_height_m", 0.0
        ).value)
        self.ground_plane_below_tolerance = float(self.declare_parameter(
            "ground_plane_below_tolerance_m", 0.12
        ).value)
        self.ground_plane_minimum_row_ratio = float(self.declare_parameter(
            "ground_plane_minimum_row_ratio", 0.56
        ).value)
        self.point_cloud_max_fps = float(self.declare_parameter(
            "point_cloud_max_fps", 5.0
        ).value)
        self.point_cloud_max_depth = float(self.declare_parameter(
            "point_cloud_max_depth_m", 3.0
        ).value)
        self.point_cloud_far_sparse_start = float(self.declare_parameter(
            "point_cloud_far_sparse_start_m", 0.0
        ).value)
        self.point_cloud_far_sparse_step = max(1, int(self.declare_parameter(
            "point_cloud_far_sparse_pixel_step", 1
        ).value))
        self.point_cloud_medium_maximum_depth = float(self.declare_parameter(
            "point_cloud_medium_max_depth_m", 0.0
        ).value)
        self.point_cloud_medium_sparse_step = max(1, int(self.declare_parameter(
            "point_cloud_medium_sparse_pixel_step", 1
        ).value))
        self.publish_split_topics = bool(self.declare_parameter(
            "publish_split_topics", True
        ).value)
        self.target_rate = float(self.declare_parameter("target_rate", 6.0).value)
        self.log_every_n = max(1, int(self.declare_parameter("log_every_n", 20).value))
        self.preprocess_threads = max(1, int(self.declare_parameter(
            "preprocess_threads", 1
        ).value))
        self.pipeline_preprocessing = bool(self.declare_parameter(
            "pipeline_preprocessing", False
        ).value)
        self.last_filter_stats = {}
        self.last_photometric_gain = 1.0
        self.last_photometric_offset = 0.0
        self.use_fixed_point_remap = bool(self.declare_parameter(
            "use_fixed_point_remap", False
        ).value)

        if not calibration_file or not Path(calibration_file).is_file():
            raise RuntimeError(f"calibration_file does not exist: {calibration_file}")
        if not model_path or not Path(model_path).is_file():
            raise RuntimeError(f"model_path does not exist: {model_path}")
        if self.output_width <= 0 or self.output_height <= 0:
            raise RuntimeError("output_width and output_height must be positive")
        if not 0.0 < self.minimum_depth < self.maximum_depth:
            raise RuntimeError("min_depth_m must be positive and less than max_depth_m")
        if self.target_rate <= 0.0:
            raise RuntimeError("target_rate must be positive")
        if (
            self.guided_upsampling_radius > 0
            and self.guided_upsampling_epsilon <= 0.0
        ):
            raise RuntimeError("guided_upsampling_epsilon must be positive")
        if self.photometric_patch_threshold < 0.0:
            raise RuntimeError("photometric_patch_threshold cannot be negative")
        if self.left_right_consistency and self.left_right_maximum_difference <= 0.0:
            raise RuntimeError(
                "left_right_maximum_difference_px must be positive when enabled"
            )
        if (
            self.left_right_consistency
            and self.left_right_model_path
            and not Path(self.left_right_model_path).is_file()
        ):
            raise RuntimeError(
                f"left_right_model_path does not exist: {self.left_right_model_path}"
            )
        if (
            self.stereo_photometric_minimum_gain <= 0.0
            or self.stereo_photometric_maximum_gain
            < self.stereo_photometric_minimum_gain
            or self.stereo_photometric_maximum_offset < 0.0
        ):
            raise RuntimeError("invalid stereo photometric normalization limits")
        if self.ground_plane_prior_enabled:
            if self.ground_plane_camera_height <= 0.0:
                raise RuntimeError(
                    "ground_plane_camera_height_m must be positive when the prior is enabled"
                )
            if self.ground_plane_below_tolerance < 0.0:
                raise RuntimeError("ground_plane_below_tolerance_m cannot be negative")
            if not 0.0 <= self.ground_plane_minimum_row_ratio < 1.0:
                raise RuntimeError(
                    "ground_plane_minimum_row_ratio must be in [0, 1)"
                )

        cv2.setNumThreads(max(1, int(self.declare_parameter("opencv_num_threads", 2).value)))
        self.preprocess_pool = (
            ThreadPoolExecutor(
                max_workers=self.preprocess_threads - 1,
                thread_name_prefix="crestereo-preprocess",
            )
            if self.preprocess_threads > 1 else None
        )
        self.session = self._create_session(model_path, provider, cache_path)
        inputs = self.session.get_inputs()
        if len(inputs) not in (2, 4):
            raise RuntimeError(f"CREStereo model must expose 2 or 4 inputs, got {len(inputs)}")
        self.input_names = [item.name for item in inputs]
        self.output_names = [item.name for item in self.session.get_outputs()]
        shape = inputs[-1].shape
        if len(shape) != 4 or not isinstance(shape[2], int) or not isinstance(shape[3], int):
            raise RuntimeError(f"CREStereo model must have a fixed NCHW shape, got {shape}")
        self.model_height, self.model_width = shape[2], shape[3]
        self.has_refinement = len(inputs) == 4
        self.io_binding = None
        self.input_ortvalues = []
        self.output_ortvalue = None
        if provider == "cuda" and self.use_cuda_io_binding:
            self._prepare_cuda_io_binding(inputs)
        self.left_right_session = None
        self.left_right_input_names = []
        self.left_right_output_names = []
        self.left_right_model_height = self.model_height
        self.left_right_model_width = self.model_width
        if self.left_right_consistency and self.left_right_model_path:
            # A half-resolution initial CREStereo pass is sufficient for the
            # reverse confidence field. It preserves the 640x360 refined
            # forward geometry while avoiding a second full 260 ms cascade.
            self.left_right_session = self._create_session(
                self.left_right_model_path,
                provider,
                cache_path,
                enable_cuda_graph=False,
            )
            reverse_inputs = self.left_right_session.get_inputs()
            if len(reverse_inputs) != 2:
                raise RuntimeError(
                    "left-right confidence model must expose exactly two inputs"
                )
            reverse_shape = reverse_inputs[-1].shape
            if (
                len(reverse_shape) != 4
                or not isinstance(reverse_shape[2], int)
                or not isinstance(reverse_shape[3], int)
            ):
                raise RuntimeError(
                    "left-right confidence model must have a fixed NCHW shape"
                )
            self.left_right_model_height = reverse_shape[2]
            self.left_right_model_width = reverse_shape[3]
            self.left_right_input_names = [item.name for item in reverse_inputs]
            self.left_right_output_names = [
                item.name for item in self.left_right_session.get_outputs()
            ]

        self._load_calibration(calibration_file)
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # RGB-D is a real-time stream, not archival data.  A one-frame,
        # best-effort output queue prevents a slow visualizer or mapper from
        # blocking inference and increasing sensor-to-odometry latency.
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        left_topic = self.declare_parameter(
            "left_image_topic", "/left_camera/image/compressed"
        ).value
        right_topic = self.declare_parameter(
            "right_image_topic", "/right_camera/image/compressed"
        ).value
        input_type = CompressedImage if self.compressed_input else Image
        self.left_subscription = self.create_subscription(
            input_type, left_topic, lambda message: self._input("left", message), input_qos
        )
        self.right_subscription = self.create_subscription(
            input_type, right_topic, lambda message: self._input("right", message), input_qos
        )
        self.color_publisher = self.create_publisher(
            Image, self.declare_parameter(
                "color_output_topic", "/usb_stereo/left/image_rect_color"
            ).value, output_qos
        )
        self.preview_publisher = self.create_publisher(
            Image, self.declare_parameter(
                "preview_output_topic", "/usb_stereo/left/image_preview"
            ).value, output_qos
        )
        self.depth_publisher = self.create_publisher(
            Image, self.declare_parameter("depth_output_topic", "/usb_stereo/depth").value,
            output_qos
        )
        self.depth_preview_publisher = self.create_publisher(
            Image, self.declare_parameter(
                "depth_preview_output_topic", "/usb_stereo/depth_preview"
            ).value, output_qos
        )
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, self.declare_parameter(
                "camera_info_output_topic", "/usb_stereo/left/camera_info"
            ).value, output_qos
        )
        self.rgbd_publisher = self.create_publisher(
            RGBDImage, self.declare_parameter(
                "rgbd_output_topic", "/usb_stereo/rgbd_image"
            ).value, output_qos
        )
        self.point_cloud_publisher = self.create_publisher(
            PointCloud2, self.declare_parameter(
                "point_cloud_output_topic", "/usb_stereo/points"
            ).value, output_qos
        )
        self.disparity_publisher = self.create_publisher(
            Image, self.declare_parameter(
                "disparity_output_topic", "/usb_stereo/disparity"
            ).value, output_qos
        )

        self.pair_lock = threading.Condition()
        self.processed_lock = threading.Condition()
        self.left_messages: Dict[int, object] = {}
        self.right_messages: Dict[int, object] = {}
        self.pending_pair: Optional[Tuple[object, object]] = None
        self.pending_processed = None
        self.stop_event = threading.Event()
        self.frames = 0
        self.dropped = 0
        self.preprocessed_dropped = 0
        self.last_point_cloud_time = 0.0
        self.last_log_time = time.monotonic()
        self.inference_sum_ms = 0.0
        self.preprocess_sum_ms = 0.0
        self.postprocess_sum_ms = 0.0
        self.processing_sum_ms = 0.0
        self.publish_sum_ms = 0.0
        self.next_inference_time = 0.0
        self.preprocess_worker = None
        if self.pipeline_preprocessing:
            self.preprocess_worker = threading.Thread(
                target=self._preprocess_worker,
                name="crestereo-preprocess-pipeline",
                # Keep native CUDA/OpenCV state alive until preprocessing has
                # actually left its worker. A daemon thread can be torn down
                # underneath ONNX Runtime during interpreter shutdown.
                daemon=False,
            )
            self.preprocess_worker.start()
        self.worker = threading.Thread(
            target=self._worker,
            name="crestereo-inference",
            daemon=False,
        )
        self.worker.start()
        self.context.on_shutdown(self._request_stop)
        active = ",".join(self.session.get_providers())
        profile = "refined" if self.has_refinement else "single-pass"
        if not self.left_right_consistency:
            confidence_profile = "off"
        elif self.left_right_session is None:
            confidence_profile = f"full {self.model_width}x{self.model_height}"
        else:
            confidence_profile = (
                f"fast {self.left_right_model_width}x{self.left_right_model_height}"
            )
        self.get_logger().info(
            f"CREStereo ready: {profile} {self.model_width}x{self.model_height}, "
            f"providers={active}, RGB-D={self.output_width}x{self.output_height}, "
            f"depth={self.minimum_depth:.1f}-{self.maximum_depth:.1f} m, "
            f"cuda_graph={'on' if self.io_binding is not None and self.enable_cuda_graph else 'off'}, "
            f"preprocess_threads={self.preprocess_threads}, "
            f"pipeline={'on' if self.pipeline_preprocessing else 'off'}, "
            f"fixed_remap={'on' if self.use_fixed_point_remap else 'off'}, "
            f"left_right={confidence_profile}, "
            f"ground_prior="
            f"{'%.2f m' % self.ground_plane_camera_height if self.ground_plane_prior_enabled else 'off'}"
        )

    def _create_session(
        self,
        model_path: str,
        provider: str,
        cache_path: str,
        enable_cuda_graph: Optional[bool] = None,
    ):
        available = ort.get_available_providers()
        if provider not in ("tensorrt", "cuda"):
            raise RuntimeError("execution_provider must be 'tensorrt' or 'cuda'")
        if provider == "tensorrt":
            if "TensorrtExecutionProvider" not in available:
                raise RuntimeError("ONNX Runtime TensorRT provider is unavailable")
            Path(cache_path).mkdir(parents=True, exist_ok=True)
            providers = [
                ("TensorrtExecutionProvider", {
                    "trt_engine_cache_enable": True,
                    "trt_engine_cache_path": cache_path,
                    "trt_timing_cache_enable": True,
                    "trt_fp16_enable": True,
                }),
                "CUDAExecutionProvider",
            ]
        else:
            if "CUDAExecutionProvider" not in available:
                raise RuntimeError("ONNX Runtime CUDA provider is unavailable")
            providers = [("CUDAExecutionProvider", {
                "enable_cuda_graph": (
                    self.enable_cuda_graph
                    if enable_cuda_graph is None else enable_cuda_graph
                ),
                "arena_extend_strategy": "kSameAsRequested",
            })]
        options = ort.SessionOptions()
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        if provider == "cuda":
            # ORT still lists CPUExecutionProvider for shape/constant work, but
            # this prevents an unsupported compute operator from silently
            # moving the learned network to CPU on a different installation.
            options.add_session_config_entry("session.disable_cpu_ep_fallback", "1")
        return ort.InferenceSession(model_path, sess_options=options, providers=providers)

    def _prepare_cuda_io_binding(self, inputs) -> None:
        """Pin fixed model tensors on CUDA for graph replay and stable latency."""
        output = self.session.get_outputs()[0]
        shapes = [list(item.shape) for item in inputs]
        output_shape = list(output.shape)
        if any(not all(isinstance(value, int) and value > 0 for value in shape)
               for shape in [*shapes, output_shape]):
            raise RuntimeError("CUDA I/O binding requires fixed model input/output shapes")
        self.input_ortvalues = [
            ort.OrtValue.ortvalue_from_shape_and_type(
                shape, np.float32, "cuda", 0
            ) for shape in shapes
        ]
        self.output_ortvalue = ort.OrtValue.ortvalue_from_shape_and_type(
            output_shape, np.float32, "cuda", 0
        )
        self.io_binding = self.session.io_binding()
        for name, value in zip(self.input_names, self.input_ortvalues):
            self.io_binding.bind_ortvalue_input(name, value)
        self.io_binding.bind_ortvalue_output(output.name, self.output_ortvalue)

    def _load_calibration(self, path: str) -> None:
        storage = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        if not storage.isOpened():
            raise RuntimeError(f"could not open calibration: {path}")
        original_width = int(storage.getNode("image_width").real())
        original_height = int(storage.getNode("image_height").real())
        k_left = _read_matrix(storage, "camera_matrix_left", (3, 3))
        d_left = _read_matrix(storage, "distortion_left", (1, 5))
        k_right = _read_matrix(storage, "camera_matrix_right", (3, 3))
        d_right = _read_matrix(storage, "distortion_right", (1, 5))
        r_left = _read_matrix(storage, "rectification_left", (3, 3))
        r_right = _read_matrix(storage, "rectification_right", (3, 3))
        p_left = _read_matrix(storage, "projection_left", (3, 4))
        p_right = _read_matrix(storage, "projection_right", (3, 4))
        storage.release()
        self.original_size = (original_width, original_height)
        model_projection_left = _scale_projection(
            p_left, self.model_width / original_width, self.model_height / original_height
        )
        model_projection_right = _scale_projection(
            p_right, self.model_width / original_width, self.model_height / original_height
        )
        self.output_projection = _scale_projection(
            p_left, self.output_width / original_width, self.output_height / original_height
        )
        self.model_projection = model_projection_left
        self.model_fx = float(model_projection_left[0, 0])
        self.baseline = abs(float(p_right[0, 3] / p_right[0, 0]))
        left_model_maps = cv2.initUndistortRectifyMap(
            k_left, d_left, r_left, model_projection_left[:, :3],
            (self.model_width, self.model_height), cv2.CV_32FC1
        )
        right_model_maps = cv2.initUndistortRectifyMap(
            k_right, d_right, r_right, model_projection_right[:, :3],
            (self.model_width, self.model_height), cv2.CV_32FC1
        )
        left_output_maps = cv2.initUndistortRectifyMap(
            k_left, d_left, r_left, self.output_projection[:, :3],
            (self.output_width, self.output_height), cv2.CV_32FC1
        )
        source_valid = np.full((original_height, original_width), 255, dtype=np.uint8)
        self.left_model_valid = cv2.remap(
            source_valid, *left_model_maps, cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT, borderValue=0
        ) > 0
        self.right_model_valid = cv2.remap(
            source_valid, *right_model_maps, cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT, borderValue=0
        ) > 0
        if self.use_fixed_point_remap:
            # OpenCV's fixed-point map format uses the same interpolation
            # table as INTER_LINEAR, but avoids three large floating-point
            # map reads for every 1080p stereo frame on the NX CPU.
            self.left_model_maps = cv2.convertMaps(
                *left_model_maps, cv2.CV_16SC2
            )
            self.right_model_maps = cv2.convertMaps(
                *right_model_maps, cv2.CV_16SC2
            )
            self.left_output_maps = cv2.convertMaps(
                *left_output_maps, cv2.CV_16SC2
            )
        else:
            self.left_model_maps = left_model_maps
            self.right_model_maps = right_model_maps
            self.left_output_maps = left_output_maps

    @staticmethod
    def _stamp_key(message) -> int:
        return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)

    def _input(self, side: str, message) -> None:
        key = self._stamp_key(message)
        with self.pair_lock:
            own = self.left_messages if side == "left" else self.right_messages
            other = self.right_messages if side == "left" else self.left_messages
            own[key] = message
            if key in other:
                left = message if side == "left" else other.pop(key)
                right = other.pop(key) if side == "left" else message
                own.pop(key, None)
                if self.pending_pair is not None:
                    self.dropped += 1
                self.pending_pair = (left, right)
                self.pair_lock.notify()
            cutoff = key - 1_000_000_000
            for cache in (self.left_messages, self.right_messages):
                for old_key in [candidate for candidate in cache if candidate < cutoff]:
                    cache.pop(old_key, None)

    def _decode(self, message) -> np.ndarray:
        if self.compressed_input:
            image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError("could not decode compressed stereo image")
            return image
        if message.encoding not in ("bgr8", "rgb8", "mono8"):
            raise RuntimeError(f"unsupported raw image encoding: {message.encoding}")
        channels = 1 if message.encoding == "mono8" else 3
        raw = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.step
        )[:, : message.width * channels]
        image = raw.reshape(message.height, message.width, channels).copy()
        if message.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif message.encoding == "mono8":
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        return image

    @staticmethod
    def _remap(image: np.ndarray, maps) -> np.ndarray:
        return cv2.remap(
            image, *maps, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT
        )

    def _preprocess_pair(self, pair) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Decode and rectify a pair without serializing independent CPU work."""
        if self.preprocess_pool is None:
            left_source = self._decode(pair[0])
            right_source = self._decode(pair[1])
        else:
            left_future = self.preprocess_pool.submit(self._decode, pair[0])
            right_source = self._decode(pair[1])
            left_source = left_future.result()
        if (
            left_source.shape[1::-1] != self.original_size
            or right_source.shape[1::-1] != self.original_size
        ):
            raise RuntimeError(
                f"input size {left_source.shape[1]}x{left_source.shape[0]} does not match "
                f"calibration {self.original_size[0]}x{self.original_size[1]}"
            )
        if self.preprocess_pool is None:
            left_model = self._remap(left_source, self.left_model_maps)
            right_model = self._remap(right_source, self.right_model_maps)
            left_output = self._remap(left_source, self.left_output_maps)
        elif self.preprocess_threads >= 3:
            left_future = self.preprocess_pool.submit(
                self._remap, left_source, self.left_model_maps
            )
            right_future = self.preprocess_pool.submit(
                self._remap, right_source, self.right_model_maps
            )
            left_output = self._remap(left_source, self.left_output_maps)
            left_model = left_future.result()
            right_model = right_future.result()
        else:
            left_future = self.preprocess_pool.submit(
                self._remap, left_source, self.left_model_maps
            )
            right_model = self._remap(right_source, self.right_model_maps)
            left_model = left_future.result()
            left_output = self._remap(left_source, self.left_output_maps)
        if self.stereo_photometric_normalization:
            right_model, gain, offset = _normalize_stereo_photometry(
                left_model,
                right_model,
                self.left_model_valid & self.right_model_valid,
                self.stereo_photometric_minimum_gain,
                self.stereo_photometric_maximum_gain,
                self.stereo_photometric_maximum_offset,
            )
            self.last_photometric_gain = gain
            self.last_photometric_offset = offset
        return left_model, right_model, left_output

    def _preprocess_worker(self) -> None:
        """Keep one fresh rectified pair ready while CUDA handles the current frame."""
        while not self.stop_event.is_set() and rclpy.ok():
            with self.pair_lock:
                while self.pending_pair is None and not self.stop_event.is_set():
                    self.pair_lock.wait(timeout=0.5)
                pair, self.pending_pair = self.pending_pair, None
            if pair is None:
                continue
            try:
                started = time.monotonic()
                left_model, right_model, left_output = self._preprocess_pair(pair)
                processed = (
                    pair[0].header,
                    left_model,
                    right_model,
                    left_output,
                    (time.monotonic() - started) * 1000.0,
                    started,
                )
                with self.processed_lock:
                    if self.pending_processed is not None:
                        self.preprocessed_dropped += 1
                    self.pending_processed = processed
                    self.processed_lock.notify()
            except Exception as error:
                if not rclpy.ok() or self.stop_event.is_set():
                    break
                self.get_logger().error(f"CREStereo preprocessing failed: {error}")

    @staticmethod
    def _tensor(image: np.ndarray) -> np.ndarray:
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32)

    def _infer(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        left_tensor, right_tensor = self._tensor(left), self._tensor(right)
        if self.has_refinement:
            half_size = (self.model_width // 2, self.model_height // 2)
            left_half = self._tensor(cv2.resize(left, half_size, interpolation=cv2.INTER_AREA))
            right_half = self._tensor(cv2.resize(right, half_size, interpolation=cv2.INTER_AREA))
            tensors = (left_half, right_half, left_tensor, right_tensor)
        else:
            tensors = (left_tensor, right_tensor)
        if self.io_binding is not None:
            for value, tensor in zip(self.input_ortvalues, tensors):
                value.update_inplace(tensor)
            self.session.run_with_iobinding(self.io_binding)
            output = self.output_ortvalue.numpy()
        else:
            output = self.session.run(
                self.output_names, dict(zip(self.input_names, tensors))
            )[0]
        if output.ndim != 4 or output.shape[1] < 1:
            raise RuntimeError(f"unexpected CREStereo output shape: {output.shape}")
        return output[0]

    def _infer_reverse_disparity(
        self, left: np.ndarray, right: np.ndarray
    ) -> np.ndarray:
        """Infer positive right-to-left disparity in flipped coordinates."""
        flipped_right = cv2.flip(right, 1)
        flipped_left = cv2.flip(left, 1)
        if self.left_right_session is None:
            return self._infer(flipped_right, flipped_left)[0]

        reverse_size = (
            self.left_right_model_width, self.left_right_model_height
        )
        reverse_right = cv2.resize(
            flipped_right, reverse_size, interpolation=cv2.INTER_AREA
        )
        reverse_left = cv2.resize(
            flipped_left, reverse_size, interpolation=cv2.INTER_AREA
        )
        tensors = (
            self._tensor(reverse_right),
            self._tensor(reverse_left),
        )
        output = self.left_right_session.run(
            self.left_right_output_names,
            dict(zip(self.left_right_input_names, tensors)),
        )[0]
        if output.ndim != 4 or output.shape[1] < 1:
            raise RuntimeError(
                f"unexpected left-right confidence output shape: {output.shape}"
            )
        disparity = np.asarray(output[0, 0], dtype=np.float32)
        if disparity.shape != (self.model_height, self.model_width):
            disparity = cv2.resize(
                disparity,
                (self.model_width, self.model_height),
                interpolation=cv2.INTER_LINEAR,
            ) * (self.model_width / self.left_right_model_width)
        return disparity

    def _filter_depth(
        self,
        flow: np.ndarray,
        reverse_disparity: Optional[np.ndarray],
        left: np.ndarray,
        right: np.ndarray,
        output_guide: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        disparity = np.asarray(flow[0], dtype=np.float32)
        with np.errstate(divide="ignore", invalid="ignore"):
            depth = self.model_fx * self.baseline / disparity
        valid = (
            np.isfinite(disparity)
            & (disparity > 0.25)
            & np.isfinite(depth)
            & (depth >= self.minimum_depth)
            & (depth <= self.maximum_depth)
            & self.left_model_valid
        )
        filter_stats = {
            "range": float(np.count_nonzero(valid)) / valid.size,
        }
        x_right = np.rint(
            np.arange(self.model_width, dtype=np.float32)[None, :] - disparity
        ).astype(np.int32)
        inside = (x_right >= 0) & (x_right < self.model_width)
        clipped = np.clip(x_right, 0, self.model_width - 1)
        rows = np.arange(self.model_height)[:, None]
        valid &= inside & self.right_model_valid[rows, clipped]
        filter_stats["rect"] = float(np.count_nonzero(valid)) / valid.size
        if flow.shape[0] > 1 and self.maximum_vertical_flow > 0.0:
            valid &= np.abs(flow[1]) <= self.maximum_vertical_flow
        filter_stats["vflow"] = float(np.count_nonzero(valid)) / valid.size
        if reverse_disparity is not None:
            valid, _ = _left_right_consistency_mask(
                disparity,
                reverse_disparity,
                valid,
                self.left_right_maximum_difference,
            )
        filter_stats["lr"] = float(np.count_nonzero(valid)) / valid.size
        if self.photometric_threshold > 0.0:
            valid, _, _ = _photometric_consistency_mask(
                left,
                right,
                disparity,
                valid,
                self.photometric_threshold,
                self.photometric_patch_size,
                self.photometric_patch_threshold,
            )
        filter_stats["photo"] = float(np.count_nonzero(valid)) / valid.size
        if self.far_artifact_depth > 0.0 and self.far_support_size > 1:
            support = cv2.boxFilter(
                valid.astype(np.uint8), cv2.CV_16U,
                (self.far_support_size, self.far_support_size), normalize=False
            )
            valid &= (depth < self.far_artifact_depth) | (support >= self.far_minimum_support)
        filter_stats["far"] = float(np.count_nonzero(valid)) / valid.size
        depth[~valid] = 0.0
        valid_ratio = float(np.count_nonzero(valid)) / valid.size
        depth_mm_model = np.clip(depth * 1000.0, 0.0, 65535.0).astype(np.uint16)
        ground_corrected_model = np.zeros(depth_mm_model.shape, dtype=bool)
        if self.ground_plane_prior_enabled:
            # Apply the geometric constraint on CREStereo's native grid. The
            # output image is nearest-neighbor expanded from this grid, so
            # doing the same work after 3x upsampling would cost about 9x more
            # without adding any learned spatial detail.
            depth_mm_model, ground_corrected_model = _apply_ground_plane_prior(
                depth_mm_model,
                self.model_projection,
                self.ground_plane_camera_height,
                self.ground_plane_below_tolerance,
                self.ground_plane_minimum_row_ratio,
                self.minimum_depth,
                self.maximum_depth,
            )
        if self.guided_upsampling_radius > 0:
            model_valid = depth_mm_model > 0
            model_disparity = np.zeros(depth_mm_model.shape, dtype=np.float32)
            model_disparity[model_valid] = (
                self.model_fx * self.baseline
                / (depth_mm_model[model_valid].astype(np.float32) * 0.001)
            )
            guided_width = (
                min(self.guided_upsampling_width, self.output_width)
                if self.guided_upsampling_width > 0
                else self.output_width
            )
            guided_height = max(
                1, round(self.output_height * guided_width / self.output_width)
            )
            guided_size = (guided_width, guided_height)
            guided_image = (
                output_guide
                if guided_size == (self.output_width, self.output_height)
                else cv2.resize(output_guide, guided_size, interpolation=cv2.INTER_AREA)
            )
            disparity_output = _edge_aware_upsample_disparity(
                model_disparity,
                model_valid,
                guided_image,
                guided_size,
                guided_width / self.model_width,
                self.guided_upsampling_radius,
                self.guided_upsampling_epsilon,
            )
            if guided_size != (self.output_width, self.output_height):
                disparity_output = cv2.resize(
                    disparity_output,
                    (self.output_width, self.output_height),
                    interpolation=cv2.INTER_LINEAR,
                ) * (self.output_width / guided_width)
                output_valid = cv2.resize(
                    model_valid.astype(np.uint8),
                    (self.output_width, self.output_height),
                    interpolation=cv2.INTER_NEAREST,
                ).astype(bool)
                disparity_output[~output_valid] = 0.0
            depth_output = np.zeros(disparity_output.shape, dtype=np.float32)
            output_valid = disparity_output > 0.0
            depth_output[output_valid] = (
                float(self.output_projection[0, 0]) * self.baseline
                / disparity_output[output_valid]
            )
            output_valid &= (
                np.isfinite(depth_output)
                & (depth_output >= self.minimum_depth)
                & (depth_output <= self.maximum_depth)
            )
            depth_mm = np.zeros(disparity_output.shape, dtype=np.uint16)
            depth_mm[output_valid] = np.clip(
                np.rint(depth_output[output_valid] * 1000.0), 0.0, 65535.0
            ).astype(np.uint16)
        else:
            depth_mm = cv2.resize(
                depth_mm_model, (self.output_width, self.output_height),
                interpolation=cv2.INTER_NEAREST
            )
        if self.median_size >= 3 and self.median_max_difference_m > 0.0:
            median = cv2.medianBlur(depth_mm, self.median_size)
            difference = cv2.absdiff(depth_mm, median)
            supported = (
                (depth_mm > 0) & (median > 0)
                & (difference <= self.median_max_difference_m * 1000.0)
            )
            depth_mm[~supported] = 0
            depth_mm[supported] = median[supported]
        ground_corrected = cv2.resize(
            ground_corrected_model.astype(np.uint8),
            (self.output_width, self.output_height),
            interpolation=cv2.INTER_NEAREST,
        ).astype(bool)
        # The edge-preserving output median may invalidate a corrected model
        # cell; do not claim or rewrite disparity for those zero-depth pixels.
        ground_corrected &= depth_mm > 0
        filter_stats["ground"] = (
            float(np.count_nonzero(ground_corrected)) / depth_mm.size
        )
        filter_stats["output"] = (
            float(np.count_nonzero(depth_mm)) / depth_mm.size
        )
        self.last_filter_stats = filter_stats
        disparity_output = np.full(depth_mm.shape, -1.0, dtype=np.float32)
        final_valid = depth_mm > 0
        disparity_output[final_valid] = (
            float(self.output_projection[0, 0]) * self.baseline
            / (depth_mm[final_valid].astype(np.float32) * 0.001)
        )
        return depth_mm, disparity_output.astype(np.float32), valid_ratio

    def _worker(self) -> None:
        while not self.stop_event.is_set() and rclpy.ok():
            if self.pipeline_preprocessing:
                with self.processed_lock:
                    while not self.stop_event.is_set():
                        delay = self.next_inference_time - time.monotonic()
                        if self.pending_processed is not None and delay <= 0.0:
                            break
                        self.processed_lock.wait(
                            timeout=max(0.001, min(0.5, delay))
                            if delay > 0.0 else 0.5
                        )
                    processed, self.pending_processed = self.pending_processed, None
                if processed is None:
                    continue
                (
                    input_header,
                    left_model,
                    right_model,
                    left_output,
                    preprocess_ms,
                    started,
                ) = processed
            else:
                with self.pair_lock:
                    while not self.stop_event.is_set():
                        delay = self.next_inference_time - time.monotonic()
                        if self.pending_pair is not None and delay <= 0.0:
                            break
                        # New pairs wake this wait and replace the pending slot;
                        # inference always takes the newest pair at the next slot.
                        self.pair_lock.wait(
                            timeout=max(0.001, min(0.5, delay))
                            if delay > 0.0 else 0.5
                        )
                    pair, self.pending_pair = self.pending_pair, None
                if pair is None:
                    continue
            try:
                if not self.pipeline_preprocessing:
                    started = time.monotonic()
                    left_model, right_model, left_output = self._preprocess_pair(pair)
                    input_header = pair[0].header
                    preprocess_ms = (time.monotonic() - started) * 1000.0
                # In pipeline mode ``started`` is the beginning of this
                # frame's preprocessing, so include the overlapped CPU stage
                # in the rate period. Starting a fresh period only after CPU
                # preprocessing would add avoidable sensor-to-depth latency.
                rate_epoch = started if self.pipeline_preprocessing else time.monotonic()
                self.next_inference_time = rate_epoch + 1.0 / self.target_rate
                inference_started = time.monotonic()
                flow = self._infer(left_model, right_model)
                reverse_disparity = None
                if self.left_right_consistency:
                    reverse_disparity = self._infer_reverse_disparity(
                        left_model, right_model
                    )
                inference_ms = (time.monotonic() - inference_started) * 1000.0
                postprocess_started = time.monotonic()
                depth_mm, disparity, valid_ratio = self._filter_depth(
                    flow, reverse_disparity, left_model, right_model, left_output
                )
                postprocess_ms = (time.monotonic() - postprocess_started) * 1000.0
                publish_started = time.monotonic()
                self._publish(input_header, left_output, depth_mm, disparity)
                publish_ms = (time.monotonic() - publish_started) * 1000.0
                processing_ms = (time.monotonic() - started) * 1000.0
                self.frames += 1
                self.inference_sum_ms += inference_ms
                self.preprocess_sum_ms += preprocess_ms
                self.postprocess_sum_ms += postprocess_ms
                self.processing_sum_ms += processing_ms
                self.publish_sum_ms += publish_ms
                if self.frames % self.log_every_n == 0:
                    now = time.monotonic()
                    fps = self.log_every_n / max(now - self.last_log_time, 1e-6)
                    stages = self.last_filter_stats
                    self.get_logger().info(
                        f"CREStereo: {fps:.2f} Hz, inference "
                        f"{self.inference_sum_ms / self.log_every_n:.1f} ms, total "
                        f"{self.processing_sum_ms / self.log_every_n:.1f} ms, "
                        f"pre/post {self.preprocess_sum_ms / self.log_every_n:.1f}/"
                        f"{self.postprocess_sum_ms / self.log_every_n:.1f} ms, "
                        f"publish {self.publish_sum_ms / self.log_every_n:.1f} ms, "
                        f"valid {valid_ratio * 100.0:.1f}%, dropped "
                        f"{self.dropped}/{self.preprocessed_dropped}, filter "
                        f"range/rect/vflow/lr/photo/far/ground/out "
                        f"{stages.get('range', 0.0) * 100.0:.1f}/"
                        f"{stages.get('rect', 0.0) * 100.0:.1f}/"
                        f"{stages.get('vflow', 0.0) * 100.0:.1f}/"
                        f"{stages.get('lr', 0.0) * 100.0:.1f}/"
                        f"{stages.get('photo', 0.0) * 100.0:.1f}/"
                        f"{stages.get('far', 0.0) * 100.0:.1f}/"
                        f"{stages.get('ground', 0.0) * 100.0:.1f}/"
                        f"{stages.get('output', 0.0) * 100.0:.1f}%, "
                        f"stereo affine {self.last_photometric_gain:.3f}/"
                        f"{self.last_photometric_offset:+.1f}"
                    )
                    self.inference_sum_ms = 0.0
                    self.preprocess_sum_ms = 0.0
                    self.postprocess_sum_ms = 0.0
                    self.processing_sum_ms = 0.0
                    self.publish_sum_ms = 0.0
                    self.last_log_time = now
            except Exception as error:  # keep the experimental chain diagnosable
                if not rclpy.ok() or self.stop_event.is_set():
                    break
                self.get_logger().error(f"CREStereo frame failed: {error}")

    def _publish(
        self,
        input_header: Header,
        color: np.ndarray,
        depth_mm: np.ndarray,
        disparity: np.ndarray,
    ) -> None:
        header = Header()
        header.stamp = input_header.stamp
        header.frame_id = self.output_frame
        color_message = _image_message(header, "bgr8", color)
        depth_message = _image_message(header, "16UC1", depth_mm)
        info = _camera_info(header, (self.output_width, self.output_height), self.output_projection)
        rgbd = RGBDImage()
        rgbd.header = header
        rgbd.rgb = color_message
        rgbd.depth = depth_message
        rgbd.rgb_camera_info = info
        rgbd.depth_camera_info = info
        self.rgbd_publisher.publish(rgbd)
        if self.publish_split_topics and self.color_publisher.get_subscription_count():
            self.color_publisher.publish(color_message)
        if self.publish_split_topics and self.depth_publisher.get_subscription_count():
            self.depth_publisher.publish(depth_message)
        if self.publish_split_topics and self.camera_info_publisher.get_subscription_count():
            self.camera_info_publisher.publish(info)
        if self.preview_publisher.get_subscription_count():
            # Reuse the already rectified output instead of resizing the full
            # 1080p source again when RViz is open.
            self.preview_publisher.publish(color_message)
        if self.depth_preview_publisher.get_subscription_count():
            self.depth_preview_publisher.publish(depth_message)
        if self.disparity_publisher.get_subscription_count():
            self.disparity_publisher.publish(_image_message(header, "32FC1", disparity))
        now = time.monotonic()
        point_cloud_due = (
            self.point_cloud_max_fps <= 0.0
            or now - self.last_point_cloud_time >= 1.0 / self.point_cloud_max_fps
        )
        if self.point_cloud_publisher.get_subscription_count() and point_cloud_due:
            self.point_cloud_publisher.publish(_point_cloud_message(
                header, color, depth_mm, self.output_projection,
                self.point_cloud_max_depth, self.point_cloud_far_sparse_start,
                self.point_cloud_far_sparse_step,
                self.point_cloud_medium_maximum_depth,
                self.point_cloud_medium_sparse_step,
            ))
            self.last_point_cloud_time = now

    def _request_stop(self) -> None:
        self.stop_event.set()
        if hasattr(self, "pair_lock"):
            with self.pair_lock:
                self.pending_pair = None
                self.pair_lock.notify_all()
        if hasattr(self, "processed_lock"):
            with self.processed_lock:
                self.pending_processed = None
                self.processed_lock.notify_all()

    def destroy_node(self) -> bool:
        self._request_stop()
        # ROS launch escalates SIGINT after five seconds. Share one bounded
        # cleanup budget between both workers instead of waiting three seconds
        # for each in series. Normal CUDA inference and remap work finish well
        # inside this window, so ORT is still never destroyed under a worker.
        deadline = time.monotonic() + 4.0
        workers = [getattr(self, "worker", None)]
        workers.append(getattr(self, "preprocess_worker", None))
        for worker in workers:
            while (
                worker is not None
                and worker.is_alive()
                and time.monotonic() < deadline
            ):
                try:
                    worker.join(timeout=max(
                        0.0, min(0.25, deadline - time.monotonic())
                    ))
                except KeyboardInterrupt:
                    continue
        if hasattr(self, "preprocess_pool") and self.preprocess_pool is not None:
            # Workers above wait for every submitted decode/remap future, so
            # no inference thread can still be using this pool. Joining its
            # native OpenCV threads before Python begins destroying the CUDA
            # session prevents the intermittent ``terminate called without
            # an active exception`` abort observed on launch shutdown.
            self.preprocess_pool.shutdown(wait=True, cancel_futures=True)
            self.preprocess_pool = None
        # ONNX Runtime owns native CUDA worker/allocator state. Releasing both
        # sessions explicitly while Python, CUDA and ROS are still alive is
        # safer than leaving their destructors to interpreter shutdown. This
        # matters with the added reverse confidence session after a long run;
        # otherwise ORT can abort with ``terminate called without an active
        # exception`` even though both Python pipeline workers have stopped.
        self.io_binding = None
        self.output_ortvalue = None
        self.input_ortvalues = []
        self.left_right_session = None
        self.session = None
        gc.collect()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CREStereoDepthNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                # launch can deliver a second SIGINT while rclpy entities are
                # already being destroyed. The worker has been joined above.
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
