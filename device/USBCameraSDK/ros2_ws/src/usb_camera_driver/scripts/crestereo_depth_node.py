#!/usr/bin/env python3
"""Learned RGB-D frontend for the calibrated USB stereo camera.

The default backend remains the existing CREStereo ONNX chain.  An explicit
``fast_foundation_stereo`` model family can instead execute a serialized
TensorRT engine directly, while sharing the same confidence, temporal and
edge-aware output stages.  The node keeps only the newest synchronized stereo
pair while inference is busy so RTAB-Map receives fresh frames instead of an
ever-growing backlog.
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


ALGORITHM_REVISION = "confidence_v2"
FAST_FOUNDATION_STEREO_REVISION = "ffs_trt_half_v1"
MODEL_FAMILY_REVISIONS = {
    "crestereo": ALGORITHM_REVISION,
    "fast_foundation_stereo": FAST_FOUNDATION_STEREO_REVISION,
}
_FFS_IMAGENET_MEAN = np.array((0.485, 0.456, 0.406), dtype=np.float32).reshape(
    1, 3, 1, 1
)
_FFS_IMAGENET_STD = np.array((0.229, 0.224, 0.225), dtype=np.float32).reshape(
    1, 3, 1, 1
)


def _fast_foundation_stereo_tensor(
    image: np.ndarray,
    content_size: Tuple[int, int] = (320, 180),
    engine_size: Tuple[int, int] = (320, 192),
) -> Tuple[np.ndarray, int]:
    """Create the externally normalized, letterboxed FFS engine input.

    The half-resolution deployment keeps the calibrated 16:9 image content at
    320x180.  The exported TensorRT graph is 320x192, so six replicated rows
    are added above and below exactly like Fast-FoundationStereo's
    ``InputPadder``.  Replication happens after ImageNet normalization and is
    numerically equivalent to padding the RGB image before normalization.
    """
    source = np.asarray(image)
    if source.ndim != 3 or source.shape[2] != 3:
        raise ValueError("Fast-FoundationStereo input must be a BGR image")
    content_width, content_height = content_size
    engine_width, engine_height = engine_size
    if min(content_width, content_height, engine_width, engine_height) <= 0:
        raise ValueError("Fast-FoundationStereo dimensions must be positive")
    if engine_width != content_width or engine_height < content_height:
        raise ValueError(
            "Fast-FoundationStereo engine must preserve content width and pad height"
        )
    padding = engine_height - content_height
    if padding % 2:
        raise ValueError("Fast-FoundationStereo vertical padding must be symmetric")
    if source.shape[1::-1] != content_size:
        source = cv2.resize(source, content_size, interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(source, cv2.COLOR_BGR2RGB)
    tensor = np.ascontiguousarray(
        rgb.transpose(2, 0, 1)[None], dtype=np.float32
    )
    tensor = (tensor / 255.0 - _FFS_IMAGENET_MEAN) / _FFS_IMAGENET_STD
    pad_top = padding // 2
    if padding:
        tensor = np.pad(
            tensor,
            ((0, 0), (0, 0), (pad_top, padding - pad_top), (0, 0)),
            mode="edge",
        )
    return np.ascontiguousarray(tensor, dtype=np.float32), pad_top


def _crop_fast_foundation_stereo_output(
    output: np.ndarray,
    content_size: Tuple[int, int],
    pad_top: int,
) -> np.ndarray:
    """Crop TensorRT's padded NCHW disparity back to calibrated content."""
    flow = np.asarray(output)
    content_width, content_height = content_size
    if flow.ndim != 4 or flow.shape[0] != 1 or flow.shape[1] < 1:
        raise RuntimeError(
            f"unexpected Fast-FoundationStereo output shape: {flow.shape}"
        )
    if flow.shape[3] != content_width:
        raise RuntimeError(
            "Fast-FoundationStereo output width does not match calibrated content"
        )
    end = pad_top + content_height
    if pad_top < 0 or end > flow.shape[2]:
        raise RuntimeError(
            "Fast-FoundationStereo output cannot be cropped to calibrated content"
        )
    return np.ascontiguousarray(flow[0, :, pad_top:end, :], dtype=np.float32)


class _TensorRTDirectEngine:
    """Small fixed-shape TensorRT runner backed by CUDA PyTorch buffers.

    JetPack provides TensorRT and CUDA-enabled PyTorch but this deployment does
    not install PyCUDA or cuda-python.  Reusing PyTorch device allocations
    avoids another CUDA binding dependency and still passes raw device
    addresses directly to TensorRT 10's named-I/O API.
    """

    def __init__(self, path: str) -> None:
        try:
            import tensorrt as trt
            import torch
        except ImportError as error:
            raise RuntimeError(
                "direct TensorRT execution requires JetPack TensorRT and CUDA PyTorch"
            ) from error
        if not torch.cuda.is_available():
            raise RuntimeError("direct TensorRT execution requires a CUDA device")
        self._trt = trt
        self._torch = torch
        self._logger = trt.Logger(trt.Logger.WARNING)
        self._runtime = trt.Runtime(self._logger)
        with open(path, "rb") as stream:
            self._engine = self._runtime.deserialize_cuda_engine(stream.read())
        if self._engine is None:
            raise RuntimeError(f"could not deserialize TensorRT engine: {path}")
        self._context = self._engine.create_execution_context()
        if self._context is None:
            raise RuntimeError("could not create TensorRT execution context")
        self.input_names = []
        self.output_names = []
        self.shapes = {}
        self._buffers = {}
        for index in range(self._engine.num_io_tensors):
            name = self._engine.get_tensor_name(index)
            shape = tuple(int(value) for value in self._engine.get_tensor_shape(name))
            if not shape or any(value <= 0 for value in shape):
                raise RuntimeError(
                    f"TensorRT direct runner requires fixed positive shape for {name}: {shape}"
                )
            dtype = self._engine.get_tensor_dtype(name)
            if dtype == trt.float32:
                torch_dtype = torch.float32
            elif dtype == trt.float16:
                torch_dtype = torch.float16
            else:
                raise RuntimeError(
                    f"unsupported TensorRT tensor dtype for {name}: {dtype}"
                )
            mode = self._engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self.input_names.append(name)
            else:
                self.output_names.append(name)
            self.shapes[name] = shape
            self._buffers[name] = torch.empty(
                shape, dtype=torch_dtype, device="cuda"
            )
            if not self._context.set_tensor_address(
                name, int(self._buffers[name].data_ptr())
            ):
                raise RuntimeError(f"could not bind TensorRT tensor {name!r}")
        if len(self.output_names) != 1:
            raise RuntimeError(
                "Fast-FoundationStereo TensorRT engine must expose one output"
            )
        self._stream = torch.cuda.Stream()

    def run(self, tensors: Tuple[np.ndarray, ...]) -> np.ndarray:
        if len(tensors) != len(self.input_names):
            raise RuntimeError(
                f"TensorRT engine expects {len(self.input_names)} inputs, got {len(tensors)}"
            )
        with self._torch.cuda.stream(self._stream):
            for name, value in zip(self.input_names, tensors):
                source = np.ascontiguousarray(value)
                if source.shape != self.shapes[name]:
                    raise RuntimeError(
                        f"TensorRT input {name!r} expects {self.shapes[name]}, got {source.shape}"
                    )
                self._buffers[name].copy_(self._torch.from_numpy(source))
            if not self._context.execute_async_v3(
                stream_handle=self._stream.cuda_stream
            ):
                raise RuntimeError("TensorRT execute_async_v3 failed")
        self._stream.synchronize()
        return self._buffers[self.output_names[0]].cpu().numpy().copy()

    def close(self) -> None:
        self._buffers.clear()
        self._stream = None
        self._context = None
        self._engine = None
        self._runtime = None


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


def _local_disparity_consistency(
    disparity: np.ndarray,
    valid: np.ndarray,
    kernel_size: int,
    absolute_tolerance_px: float,
    relative_tolerance: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Reject locally isolated low disparity without flattening real slopes."""
    flow = np.asarray(disparity, dtype=np.float32)
    mask = np.asarray(valid, dtype=bool).copy()
    if flow.ndim != 2 or mask.shape != flow.shape:
        raise ValueError("local disparity inputs must have matching geometry")
    if kernel_size < 1 or kernel_size % 2 == 0:
        raise ValueError("local disparity kernel must be positive and odd")
    if absolute_tolerance_px < 0.0 or relative_tolerance < 0.0:
        raise ValueError("local disparity tolerances cannot be negative")
    if kernel_size == 1:
        return mask, np.zeros_like(flow), np.ones_like(flow)
    finite = np.where(np.isfinite(flow) & (flow > 0.0), flow, 0.0)
    local_median = cv2.medianBlur(finite, kernel_size)
    tolerance = absolute_tolerance_px + relative_tolerance * np.maximum(
        local_median, 0.0
    )
    deficit = np.maximum(local_median - finite, 0.0)
    mask &= (local_median <= 0.0) | (deficit <= tolerance)
    score = np.ones_like(flow, dtype=np.float32)
    supported = local_median > 0.0
    score[supported] = np.clip(
        1.0 - deficit[supported] / np.maximum(tolerance[supported] * 2.0, 1e-3),
        0.0,
        1.0,
    )
    return mask, deficit, score


def _warp_previous_disparity(
    current_gray: np.ndarray,
    previous_gray: np.ndarray,
    previous_disparity: np.ndarray,
    previous_valid: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Warp the preceding disparity into the current frame at half scale."""
    current = np.asarray(current_gray, dtype=np.uint8)
    previous = np.asarray(previous_gray, dtype=np.uint8)
    disparity = np.asarray(previous_disparity, dtype=np.float32)
    valid = np.asarray(previous_valid, dtype=bool)
    if (
        current.shape != previous.shape
        or disparity.shape != current.shape
        or valid.shape != current.shape
    ):
        raise ValueError("temporal disparity inputs must have matching geometry")
    height, width = current.shape
    small_size = (max(16, width // 2), max(16, height // 2))
    current_small = cv2.resize(current, small_size, interpolation=cv2.INTER_AREA)
    previous_small = cv2.resize(previous, small_size, interpolation=cv2.INTER_AREA)
    flow = cv2.calcOpticalFlowFarneback(
        current_small, previous_small, None, 0.5, 3, 15, 3, 5, 1.1, 0
    )
    flow = cv2.resize(flow, (width, height), interpolation=cv2.INTER_LINEAR)
    flow[:, :, 0] *= width / small_size[0]
    flow[:, :, 1] *= height / small_size[1]
    columns, rows = np.meshgrid(
        np.arange(width, dtype=np.float32),
        np.arange(height, dtype=np.float32),
    )
    map_x = columns + flow[:, :, 0]
    map_y = rows + flow[:, :, 1]
    warped_disparity = cv2.remap(
        disparity, map_x, map_y, cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT, borderValue=0.0,
    )
    warped_valid = cv2.remap(
        valid.astype(np.uint8), map_x, map_y, cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT, borderValue=0,
    ).astype(bool)
    warped_gray = cv2.remap(
        previous, map_x, map_y, cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT, borderValue=0,
    )
    photometric_error = cv2.absdiff(current, warped_gray).astype(np.float32)
    return warped_disparity, warped_valid, photometric_error


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


def _enhance_stereo_low_light(
    left_bgr: np.ndarray,
    right_bgr: np.ndarray,
    valid_mask: np.ndarray,
    trigger_luma: float,
    target_luma: float,
    minimum_gamma: float,
) -> Tuple[np.ndarray, np.ndarray, float, float]:
    """Apply one monotonic gamma curve to both model inputs in low light.

    Independent enhancement would change the correspondence problem.  This
    helper estimates one robust luma from common rectified pixels and applies
    exactly the same LUT to both views.  The published RGB image is untouched;
    enhancement exists only to expose weak floor texture to CREStereo.
    """
    left = np.asarray(left_bgr)
    right = np.asarray(right_bgr)
    mask = np.asarray(valid_mask, dtype=bool)
    if left.shape != right.shape or left.ndim != 3 or left.shape[2] != 3:
        raise ValueError("low-light enhancement requires matching BGR images")
    if mask.shape != left.shape[:2]:
        raise ValueError("low-light enhancement mask must match image geometry")
    if not 0.0 < minimum_gamma <= 1.0 or not 0.0 < target_luma < 255.0:
        raise ValueError("invalid low-light gamma parameters")
    samples = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)[::4, ::4][mask[::4, ::4]]
    samples = samples[(samples > 1) & (samples < 254)]
    median_luma = float(np.median(samples)) if samples.size >= 128 else 0.0
    if median_luma <= 1.0 or median_luma >= trigger_luma:
        return left.copy(), right.copy(), 1.0, median_luma
    gamma = float(np.clip(
        np.log(target_luma / 255.0) / np.log(median_luma / 255.0),
        minimum_gamma,
        1.0,
    ))
    values = np.arange(256, dtype=np.float32) / 255.0
    lut = np.clip(np.rint(255.0 * np.power(values, gamma)), 0, 255).astype(np.uint8)
    return cv2.LUT(left, lut), cv2.LUT(right, lut), gamma, median_luma


def _radiometric_stereo_confidence(
    left_bgr: np.ndarray,
    right_bgr: np.ndarray,
    disparity: np.ndarray,
    valid: np.ndarray,
    window_size: int,
    minimum_stddev: float,
    minimum_horizontal_gradient: float,
    reflection_residual_threshold: float,
    reflection_highlight_delta: float,
    texture_guide_bgr: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Estimate matchability and view-dependent reflection per pixel.

    Stereo needs variation along the epipolar (horizontal) direction.  Local
    standard deviation alone incorrectly labels a horizontal edge as richly
    textured, so the texture score combines contrast with horizontal Sobel
    energy and requires support in both rectified views.  Reflection is not
    inferred from brightness alone: it additionally requires a left/right
    residual or a local-contrast mismatch after warping the right view with
    the predicted disparity.  A bright diffuse wall can therefore remain
    valid, while a highlight which moves between the two lenses is penalized.
    """
    left = np.asarray(left_bgr)
    right = np.asarray(right_bgr)
    flow = np.asarray(disparity, dtype=np.float32)
    mask = np.asarray(valid, dtype=bool)
    if left.shape != right.shape or left.shape[:2] != flow.shape:
        raise ValueError("radiometric confidence inputs must have matching geometry")
    if mask.shape != flow.shape:
        raise ValueError("radiometric confidence mask must match disparity")
    texture_guide = (
        left
        if texture_guide_bgr is None
        else np.asarray(texture_guide_bgr)
    )
    if texture_guide.shape != left.shape:
        raise ValueError("texture guide must match disparity geometry")
    if window_size < 3 or window_size % 2 == 0:
        raise ValueError("radiometric confidence window must be odd and at least three")
    if (
        minimum_stddev <= 0.0
        or minimum_horizontal_gradient <= 0.0
        or reflection_residual_threshold <= 0.0
        or reflection_highlight_delta < 0.0
    ):
        raise ValueError("invalid radiometric confidence thresholds")

    left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY).astype(np.float32)
    right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY).astype(np.float32)
    texture_guide_gray = cv2.cvtColor(
        texture_guide, cv2.COLOR_BGR2GRAY
    ).astype(np.float32)

    def local_statistics(gray: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        mean = cv2.boxFilter(
            gray, cv2.CV_32F, (window_size, window_size), normalize=True,
            borderType=cv2.BORDER_REPLICATE,
        )
        mean_square = cv2.boxFilter(
            gray * gray, cv2.CV_32F, (window_size, window_size), normalize=True,
            borderType=cv2.BORDER_REPLICATE,
        )
        stddev = np.sqrt(np.maximum(mean_square - mean * mean, 0.0))
        horizontal = np.abs(cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3))
        horizontal = cv2.boxFilter(
            horizontal, cv2.CV_32F, (window_size, window_size), normalize=True,
            borderType=cv2.BORDER_REPLICATE,
        )
        return mean, stddev, horizontal

    left_mean, left_stddev, left_horizontal = local_statistics(left_gray)
    right_mean, right_stddev, right_horizontal = local_statistics(right_gray)
    _, guide_stddev, guide_horizontal = local_statistics(texture_guide_gray)
    rows = np.broadcast_to(
        np.arange(flow.shape[0], dtype=np.float32)[:, None], flow.shape
    )
    columns = np.broadcast_to(
        np.arange(flow.shape[1], dtype=np.float32)[None, :], flow.shape
    )
    right_columns = columns - flow

    def warp(field: np.ndarray) -> np.ndarray:
        return cv2.remap(
            field, right_columns, rows, cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT, borderValue=0.0,
        )

    warped_right_mean = warp(right_mean)
    warped_right_stddev = warp(right_stddev)
    warped_right_horizontal = warp(right_horizontal)
    warped_right_gray = warp(right_gray)

    # Stereo matchability needs both local contrast and variation along the
    # horizontal epipolar direction. ``maximum`` let either signal hide the
    # absence of the other (for example, a horizontal edge has contrast but
    # no horizontal support). The geometric mean is a soft AND: both terms
    # must be present, while ordinary low-contrast texture is not subjected
    # to a second hard threshold here.
    left_texture = np.sqrt(
        np.clip(guide_stddev / minimum_stddev, 0.0, 1.0)
        * np.clip(
            guide_horizontal / minimum_horizontal_gradient, 0.0, 1.0
        )
    )
    right_texture = np.sqrt(
        np.clip(warped_right_stddev / minimum_stddev, 0.0, 1.0)
        * np.clip(
            warped_right_horizontal / minimum_horizontal_gradient, 0.0, 1.0
        )
    )
    texture_score = np.clip(
        np.minimum(left_texture, right_texture), 0.0, 1.0
    ).astype(np.float32)

    residual = left_gray - warped_right_gray
    brightness_offset = float(np.median(residual[mask])) if np.any(mask) else 0.0
    view_residual = np.abs(residual - brightness_offset)
    residual_score = np.clip(
        view_residual / reflection_residual_threshold, 0.0, 1.0
    )
    contrast_asymmetry = np.abs(left_stddev - warped_right_stddev) / np.maximum(
        left_stddev + warped_right_stddev, 1.0
    )
    valid_luma = left_mean[mask & (left_mean > 1.0) & (left_mean < 254.0)]
    scene_luma = float(np.median(valid_luma)) if valid_luma.size >= 128 else 0.0
    highlight = np.clip(
        (np.maximum(left_mean, warped_right_mean)
         - scene_luma - reflection_highlight_delta)
        / max(reflection_highlight_delta, 1.0),
        0.0,
        1.0,
    )
    reflection_score = np.clip(
        np.maximum(
            residual_score * (0.35 + 0.65 * highlight),
            contrast_asymmetry * highlight,
        ),
        0.0,
        1.0,
    ).astype(np.float32)
    outside = (right_columns < 0.0) | (right_columns > flow.shape[1] - 1.001)
    texture_score[outside] = 0.0
    reflection_score[outside] = 1.0
    return texture_score, reflection_score


def _combine_depth_confidence(
    lr_score: np.ndarray,
    photo_score: np.ndarray,
    local_score: np.ndarray,
    temporal_score: np.ndarray,
    temporal_visible: np.ndarray,
    texture_score: np.ndarray,
    reflection_score: np.ndarray,
    low_texture_threshold: float,
    reflection_threshold: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Combine geometric evidence and return joint radiometric veto masks.

    Texture is evidence about observability, not geometric proof that the
    network depth is wrong. It therefore cannot reduce confidence on its own.
    Low texture is rejected only when at least two independent geometry checks
    are also weak. A detected view-dependent reflection is stronger evidence
    and needs one weak geometry check. Pixels which pass those veto rules are
    scored from LR, photometric, local-disparity and temporal consistency,
    with only a small *joint* radiometric uncertainty penalty.
    """
    arrays = [
        np.asarray(item, dtype=np.float32)
        for item in (
            lr_score,
            photo_score,
            local_score,
            temporal_score,
            texture_score,
            reflection_score,
        )
    ]
    shape = arrays[0].shape
    if any(item.shape != shape for item in arrays[1:]):
        raise ValueError("depth confidence scores must have matching geometry")
    visible = np.asarray(temporal_visible, dtype=bool)
    if visible.shape != shape:
        raise ValueError("temporal visibility must match confidence geometry")
    if not 0.0 <= low_texture_threshold <= 1.0:
        raise ValueError("low texture threshold must be in [0, 1]")
    if not 0.0 <= reflection_threshold <= 1.0:
        raise ValueError("reflection threshold must be in [0, 1]")

    lr, photo, local, temporal, texture, reflection = arrays
    low_texture_weak_count = (
        (lr < 0.80).astype(np.uint8)
        + (photo < 0.75).astype(np.uint8)
        + (local < 0.75).astype(np.uint8)
        + (visible & (temporal < 0.80)).astype(np.uint8)
    )
    reflection_weak_count = (
        (lr < 0.90).astype(np.uint8)
        + (photo < 0.85).astype(np.uint8)
        + (local < 0.75).astype(np.uint8)
        + (visible & (temporal < 0.90)).astype(np.uint8)
    )
    low_texture_risk = (
        (texture < low_texture_threshold) & (low_texture_weak_count >= 2)
    )
    reflection_risk = (
        (reflection >= reflection_threshold) & (reflection_weak_count >= 1)
    )

    geometry_confidence = (
        0.35 * np.clip(lr, 0.0, 1.0)
        + 0.25 * np.clip(photo, 0.0, 1.0)
        + 0.20 * np.clip(local, 0.0, 1.0)
        + 0.20 * np.clip(temporal, 0.0, 1.0)
    )
    weak_fraction = low_texture_weak_count.astype(np.float32) / 4.0
    joint_radiometric_risk = np.maximum(
        (1.0 - np.clip(texture, 0.0, 1.0)) * weak_fraction,
        np.clip(reflection, 0.0, 1.0) * weak_fraction,
    )
    confidence = geometry_confidence * (1.0 - 0.25 * joint_radiometric_risk)
    return (
        np.clip(confidence, 0.0, 1.0).astype(np.float32),
        low_texture_risk,
        reflection_risk,
    )


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
    # Guide numerator and support independently. Filtering their quotient
    # would let invalid zeros dilute a valid boundary and lower disparity,
    # manufacturing a false-far fringe around every confidence hole.
    guide_gray = cv2.cvtColor(guide, cv2.COLOR_BGR2GRAY)
    weighted = cv2.ximgproc.guidedFilter(
        guide_gray, weighted * disparity_scale, radius, epsilon
    )
    weights = cv2.ximgproc.guidedFilter(
        guide_gray, weights, radius, epsilon
    )
    upsampled = np.divide(
        weighted,
        np.maximum(weights, 1e-3),
        out=np.zeros_like(weighted),
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
        self.model_family = str(self.declare_parameter(
            "model_family", "crestereo"
        ).value).strip().lower()
        if self.model_family not in MODEL_FAMILY_REVISIONS:
            raise RuntimeError(
                "model_family must be 'crestereo' or 'fast_foundation_stereo'"
            )
        self.algorithm_revision = MODEL_FAMILY_REVISIONS[self.model_family]
        self.model_display_name = (
            "Fast-FoundationStereo"
            if self.model_family == "fast_foundation_stereo"
            else "CREStereo"
        )
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
        self.low_light_enhancement = bool(self.declare_parameter(
            "low_light_enhancement", False
        ).value)
        self.low_light_trigger_luma = float(self.declare_parameter(
            "low_light_trigger_luma", 36.0
        ).value)
        self.low_light_target_luma = float(self.declare_parameter(
            "low_light_target_luma", 52.0
        ).value)
        self.low_light_minimum_gamma = float(self.declare_parameter(
            "low_light_minimum_gamma", 0.55
        ).value)
        self.local_disparity_kernel = max(1, int(self.declare_parameter(
            "local_disparity_kernel_size", 5
        ).value) | 1)
        self.local_disparity_absolute_tolerance = float(self.declare_parameter(
            "local_disparity_absolute_tolerance_px", 0.75
        ).value)
        self.local_disparity_relative_tolerance = float(self.declare_parameter(
            "local_disparity_relative_tolerance", 0.08
        ).value)
        self.radiometric_confidence = bool(self.declare_parameter(
            "radiometric_confidence", True
        ).value)
        self.radiometric_analysis_width = max(0, int(self.declare_parameter(
            "radiometric_analysis_width", 320
        ).value))
        self.texture_window_size = max(3, int(self.declare_parameter(
            "texture_window_size", 7
        ).value) | 1)
        self.texture_minimum_stddev = float(self.declare_parameter(
            "texture_minimum_stddev", 3.0
        ).value)
        self.texture_minimum_horizontal_gradient = float(self.declare_parameter(
            "texture_minimum_horizontal_gradient", 2.0
        ).value)
        self.low_texture_score_threshold = float(self.declare_parameter(
            "low_texture_score_threshold", 0.25
        ).value)
        self.reflection_residual_threshold = float(self.declare_parameter(
            "reflection_residual_threshold", 12.0
        ).value)
        self.reflection_highlight_delta = float(self.declare_parameter(
            "reflection_highlight_delta", 12.0
        ).value)
        self.reflection_score_threshold = float(self.declare_parameter(
            "reflection_score_threshold", 0.55
        ).value)
        self.minimum_depth_confidence = float(self.declare_parameter(
            "minimum_depth_confidence", 0.54
        ).value)
        self.temporal_consistency = bool(self.declare_parameter(
            "temporal_consistency", False
        ).value)
        self.temporal_minimum_depth = float(self.declare_parameter(
            "temporal_minimum_depth_m", 3.0
        ).value)
        self.temporal_absolute_tolerance = float(self.declare_parameter(
            "temporal_absolute_tolerance_px", 1.0
        ).value)
        self.temporal_relative_tolerance = float(self.declare_parameter(
            "temporal_relative_tolerance", 0.10
        ).value)
        self.temporal_photometric_threshold = float(self.declare_parameter(
            "temporal_photometric_threshold", 24.0
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
        self.last_low_light_gamma = 1.0
        self.last_input_luma = 0.0
        self.previous_temporal_gray = None
        self.previous_temporal_disparity = None
        self.previous_temporal_valid = None
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
        if (
            self.low_light_trigger_luma <= 0.0
            or self.low_light_target_luma <= 0.0
            or self.low_light_target_luma >= 255.0
            or not 0.0 < self.low_light_minimum_gamma <= 1.0
        ):
            raise RuntimeError("invalid stereo low-light enhancement limits")
        if (
            self.local_disparity_absolute_tolerance < 0.0
            or self.local_disparity_relative_tolerance < 0.0
        ):
            raise RuntimeError("local disparity tolerances cannot be negative")
        if (
            self.texture_minimum_stddev <= 0.0
            or self.texture_minimum_horizontal_gradient <= 0.0
            or not 0.0 <= self.low_texture_score_threshold <= 1.0
            or self.reflection_residual_threshold <= 0.0
            or self.reflection_highlight_delta < 0.0
            or not 0.0 <= self.reflection_score_threshold <= 1.0
            or not 0.0 <= self.minimum_depth_confidence <= 1.0
        ):
            raise RuntimeError("invalid radiometric depth confidence limits")
        if (
            self.temporal_minimum_depth < self.minimum_depth
            or self.temporal_absolute_tolerance <= 0.0
            or self.temporal_relative_tolerance < 0.0
            or self.temporal_photometric_threshold <= 0.0
        ):
            raise RuntimeError("invalid temporal disparity consistency limits")
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
        self.session = None
        self.direct_engine = None
        self.engine_height = 0
        self.engine_width = 0
        self.ffs_pad_top = 0
        if self.model_family == "fast_foundation_stereo":
            if Path(model_path).suffix.lower() != ".engine":
                raise RuntimeError(
                    "fast_foundation_stereo model_path must be a serialized .engine"
                )
            self.direct_engine = _TensorRTDirectEngine(model_path)
            if self.direct_engine.input_names != ["left_image", "right_image"]:
                raise RuntimeError(
                    "Fast-FoundationStereo engine inputs must be ordered "
                    "['left_image', 'right_image']"
                )
            if self.direct_engine.output_names != ["disparity"]:
                raise RuntimeError(
                    "Fast-FoundationStereo engine output must be ['disparity']"
                )
            input_shapes = [
                self.direct_engine.shapes[name]
                for name in self.direct_engine.input_names
            ]
            if (
                input_shapes[0] != input_shapes[1]
                or input_shapes[0] != (1, 3, 192, 320)
            ):
                raise RuntimeError(
                    "Fast-FoundationStereo engine must use fixed 1x3x192x320 inputs"
                )
            output_shape = self.direct_engine.shapes[
                self.direct_engine.output_names[0]
            ]
            if output_shape != (1, 1, 192, 320):
                raise RuntimeError(
                    "Fast-FoundationStereo engine must output fixed 1x1x192x320 disparity"
                )
            self.input_names = list(self.direct_engine.input_names)
            self.output_names = list(self.direct_engine.output_names)
            self.engine_height, self.engine_width = 192, 320
            # Keep the calibrated 16:9 content undistorted. Only the inference
            # tensor is letterboxed to the engine's multiple-of-32 height.
            self.model_height, self.model_width = 180, 320
            self.ffs_pad_top = (self.engine_height - self.model_height) // 2
            self.has_refinement = False
        else:
            self.session = self._create_session(model_path, provider, cache_path)
            inputs = self.session.get_inputs()
            if len(inputs) not in (2, 4):
                raise RuntimeError(
                    f"CREStereo model must expose 2 or 4 inputs, got {len(inputs)}"
                )
            self.input_names = [item.name for item in inputs]
            self.output_names = [item.name for item in self.session.get_outputs()]
            shape = inputs[-1].shape
            if (
                len(shape) != 4
                or not isinstance(shape[2], int)
                or not isinstance(shape[3], int)
            ):
                raise RuntimeError(
                    f"CREStereo model must have a fixed NCHW shape, got {shape}"
                )
            self.model_height, self.model_width = shape[2], shape[3]
            self.engine_height, self.engine_width = (
                self.model_height, self.model_width
            )
            self.has_refinement = len(inputs) == 4
        self.io_binding = None
        self.input_ortvalues = []
        self.output_ortvalue = None
        if (
            self.model_family == "crestereo"
            and provider == "cuda"
            and self.use_cuda_io_binding
        ):
            self._prepare_cuda_io_binding(inputs)
        self.left_right_session = None
        self.left_right_input_names = []
        self.left_right_output_names = []
        self.left_right_model_height = self.model_height
        self.left_right_model_width = self.model_width
        if self.left_right_consistency and self.left_right_model_path:
            if self.model_family == "fast_foundation_stereo":
                raise RuntimeError(
                    "fast_foundation_stereo reuses its primary engine for left-right "
                    "consistency; left_right_model_path must be empty"
                )
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
        self.raw_disparity_publisher = self.create_publisher(
            Image, self.declare_parameter(
                "raw_disparity_output_topic", "/usb_stereo/disparity_raw"
            ).value, output_qos
        )
        self.confidence_publisher = self.create_publisher(
            Image, self.declare_parameter(
                "confidence_output_topic", "/usb_stereo/depth_confidence"
            ).value, output_qos
        )
        self.validity_publisher = self.create_publisher(
            Image, self.declare_parameter(
                "validity_output_topic", "/usb_stereo/depth_validity"
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
        active = (
            "TensorRT-direct"
            if self.direct_engine is not None
            else ",".join(self.session.get_providers())
        )
        if self.direct_engine is not None:
            profile = (
                f"half-content {self.model_width}x{self.model_height}, "
                f"engine={self.engine_width}x{self.engine_height}, "
                f"pad={self.ffs_pad_top}+{self.ffs_pad_top}"
            )
        else:
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
            f"{self.model_display_name} ready: "
            f"model_family={self.model_family}, "
            f"algorithm_revision={self.algorithm_revision}, "
            f"{profile}"
            f"{'' if self.direct_engine is not None else ' %dx%d' % (self.model_width, self.model_height)}, "
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
        self.output_projection = _scale_projection(
            p_left, self.output_width / original_width, self.output_height / original_height
        )
        self.model_projection = model_projection_left
        self.model_fx = float(model_projection_left[0, 0])
        self.baseline = abs(float(p_right[0, 3] / p_right[0, 0]))
        if self.model_family == "fast_foundation_stereo":
            # Rectifying 1080p directly onto a 320x180 grid aliases away the
            # weak horizontal floor texture that FFS relies on and makes the
            # flipped reverse pass diverge. Preserve the validated 640x360
            # rectified content first, then area-downsample to the engine's
            # 320x180 content grid.
            self.model_remap_size = (640, 360)
        else:
            self.model_remap_size = (self.model_width, self.model_height)
        remap_width, remap_height = self.model_remap_size
        remap_projection_left = _scale_projection(
            p_left, remap_width / original_width, remap_height / original_height
        )
        remap_projection_right = _scale_projection(
            p_right, remap_width / original_width, remap_height / original_height
        )
        left_model_maps = cv2.initUndistortRectifyMap(
            k_left, d_left, r_left, remap_projection_left[:, :3],
            self.model_remap_size, cv2.CV_32FC1
        )
        right_model_maps = cv2.initUndistortRectifyMap(
            k_right, d_right, r_right, remap_projection_right[:, :3],
            self.model_remap_size, cv2.CV_32FC1
        )
        left_output_maps = cv2.initUndistortRectifyMap(
            k_left, d_left, r_left, self.output_projection[:, :3],
            (self.output_width, self.output_height), cv2.CV_32FC1
        )
        source_valid = np.full((original_height, original_width), 255, dtype=np.uint8)
        left_model_valid = cv2.remap(
            source_valid, *left_model_maps, cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT, borderValue=0
        )
        right_model_valid = cv2.remap(
            source_valid, *right_model_maps, cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT, borderValue=0
        )
        if self.model_remap_size != (self.model_width, self.model_height):
            content_size = (self.model_width, self.model_height)
            left_model_valid = cv2.resize(
                left_model_valid, content_size, interpolation=cv2.INTER_AREA
            )
            right_model_valid = cv2.resize(
                right_model_valid, content_size, interpolation=cv2.INTER_AREA
            )
        self.left_model_valid = left_model_valid >= 254
        self.right_model_valid = right_model_valid >= 254
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
        content_size = (self.model_width, self.model_height)
        if left_model.shape[1::-1] != content_size:
            left_model = cv2.resize(
                left_model, content_size, interpolation=cv2.INTER_AREA
            )
            right_model = cv2.resize(
                right_model, content_size, interpolation=cv2.INTER_AREA
            )
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
        if self.low_light_enhancement and self.model_family == "crestereo":
            left_model, right_model, gamma, median_luma = (
                _enhance_stereo_low_light(
                    left_model,
                    right_model,
                    self.left_model_valid & self.right_model_valid,
                    self.low_light_trigger_luma,
                    self.low_light_target_luma,
                    self.low_light_minimum_gamma,
                )
            )
            self.last_low_light_gamma = gamma
            self.last_input_luma = median_luma
        elif self.model_family == "fast_foundation_stereo":
            # FFS was trained with raw RGB followed by ImageNet
            # normalization. The CRES-specific gamma lift makes dark sensor
            # noise directional: forward disparity may remain plausible but
            # the flipped reverse pass diverges and destroys LR confidence.
            # Keep measured pixels intact and let FFS's learned features handle
            # low light. Photometric normalization above is still binocular
            # and remains safe.
            gray = cv2.cvtColor(left_model, cv2.COLOR_BGR2GRAY)
            samples = gray[(self.left_model_valid & self.right_model_valid)]
            self.last_input_luma = (
                float(np.median(samples)) if samples.size else 0.0
            )
            self.last_low_light_gamma = 1.0
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
                self.get_logger().error(
                    f"{self.model_display_name} preprocessing failed: {error}"
                )

    @staticmethod
    def _tensor(image: np.ndarray) -> np.ndarray:
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float32)

    def _infer(self, left: np.ndarray, right: np.ndarray) -> np.ndarray:
        if self.direct_engine is not None:
            engine_size = (self.engine_width, self.engine_height)
            content_size = (self.model_width, self.model_height)
            left_tensor, left_pad = _fast_foundation_stereo_tensor(
                left, content_size, engine_size
            )
            right_tensor, right_pad = _fast_foundation_stereo_tensor(
                right, content_size, engine_size
            )
            if left_pad != self.ffs_pad_top or right_pad != self.ffs_pad_top:
                raise RuntimeError(
                    "Fast-FoundationStereo preprocessing padding changed unexpectedly"
                )
            output = self.direct_engine.run((left_tensor, right_tensor))
            return _crop_fast_foundation_stereo_output(
                output, content_size, self.ffs_pad_top
            )

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
    ) -> Tuple[
        np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float
    ]:
        disparity = np.asarray(flow[0], dtype=np.float32).copy()
        raw_disparity = disparity.copy()
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
        base_valid = valid.copy()
        lr_score = np.ones(disparity.shape, dtype=np.float32)
        if reverse_disparity is not None:
            valid, lr_difference = _left_right_consistency_mask(
                disparity,
                reverse_disparity,
                valid,
                self.left_right_maximum_difference,
            )
            lr_score = np.clip(
                1.0 - lr_difference / (2.0 * self.left_right_maximum_difference),
                0.0,
                1.0,
            ).astype(np.float32)
        filter_stats["lr"] = float(np.count_nonzero(valid)) / valid.size
        photo_score = np.ones(disparity.shape, dtype=np.float32)
        photo_difference = np.zeros(disparity.shape, dtype=np.float32)
        if self.photometric_threshold > 0.0:
            valid, photo_difference, _ = _photometric_consistency_mask(
                left,
                right,
                disparity,
                valid,
                self.photometric_threshold,
                self.photometric_patch_size,
                self.photometric_patch_threshold,
            )
            photo_score = np.clip(
                1.0 - photo_difference / (2.0 * self.photometric_threshold),
                0.0,
                1.0,
            ).astype(np.float32)
        filter_stats["photo"] = float(np.count_nonzero(valid)) / valid.size
        texture_score = np.ones(disparity.shape, dtype=np.float32)
        reflection_score = np.zeros(disparity.shape, dtype=np.float32)
        if self.radiometric_confidence:
            texture_guide = output_guide
            if texture_guide.shape[:2] != disparity.shape:
                texture_guide = cv2.resize(
                    texture_guide,
                    (self.model_width, self.model_height),
                    interpolation=cv2.INTER_AREA,
                )
            analysis_width = min(
                self.model_width,
                self.radiometric_analysis_width
                if self.radiometric_analysis_width > 0
                else self.model_width,
            )
            if analysis_width < self.model_width:
                scale = analysis_width / self.model_width
                analysis_height = max(1, round(self.model_height * scale))
                analysis_size = (analysis_width, analysis_height)
                analysis_left = cv2.resize(
                    left, analysis_size, interpolation=cv2.INTER_AREA
                )
                analysis_right = cv2.resize(
                    right, analysis_size, interpolation=cv2.INTER_AREA
                )
                analysis_guide = cv2.resize(
                    texture_guide, analysis_size, interpolation=cv2.INTER_AREA
                )
                analysis_disparity = cv2.resize(
                    disparity, analysis_size, interpolation=cv2.INTER_LINEAR
                ) * scale
                analysis_valid = cv2.resize(
                    valid.astype(np.uint8),
                    analysis_size,
                    interpolation=cv2.INTER_NEAREST,
                ).astype(bool)
                analysis_window = max(
                    3, int(round(self.texture_window_size * scale)) | 1
                )
            else:
                analysis_left = left
                analysis_right = right
                analysis_guide = texture_guide
                analysis_disparity = disparity
                analysis_valid = valid
                analysis_window = self.texture_window_size
            texture_score, reflection_score = _radiometric_stereo_confidence(
                analysis_left,
                analysis_right,
                analysis_disparity,
                analysis_valid,
                analysis_window,
                self.texture_minimum_stddev,
                self.texture_minimum_horizontal_gradient,
                self.reflection_residual_threshold,
                self.reflection_highlight_delta,
                analysis_guide,
            )
            if texture_score.shape != disparity.shape:
                texture_score = cv2.resize(
                    texture_score,
                    (self.model_width, self.model_height),
                    interpolation=cv2.INTER_LINEAR,
                )
                reflection_score = cv2.resize(
                    reflection_score,
                    (self.model_width, self.model_height),
                    interpolation=cv2.INTER_LINEAR,
                )
        filter_stats["low_texture"] = (
            float(np.count_nonzero(
                valid & (texture_score < self.low_texture_score_threshold)
            )) / valid.size
        )
        filter_stats["reflection"] = (
            float(np.count_nonzero(
                valid & (reflection_score >= self.reflection_score_threshold)
            )) / valid.size
        )
        valid, _, local_score = _local_disparity_consistency(
            disparity,
            valid,
            self.local_disparity_kernel,
            self.local_disparity_absolute_tolerance,
            self.local_disparity_relative_tolerance,
        )
        filter_stats["local"] = float(np.count_nonzero(valid)) / valid.size

        temporal_score = np.ones(disparity.shape, dtype=np.float32)
        temporal_visible = np.zeros(disparity.shape, dtype=bool)
        temporal_recovered = np.zeros(disparity.shape, dtype=bool)
        current_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        if (
            self.temporal_consistency
            and self.previous_temporal_gray is not None
            and self.previous_temporal_disparity is not None
            and self.previous_temporal_valid is not None
        ):
            warped, warped_valid, warp_photo_error = _warp_previous_disparity(
                current_gray,
                self.previous_temporal_gray,
                self.previous_temporal_disparity,
                self.previous_temporal_valid,
            )
            temporal_difference = np.abs(disparity - warped)
            temporal_tolerance = (
                self.temporal_absolute_tolerance
                + self.temporal_relative_tolerance
                * np.maximum(disparity, warped)
            )
            temporal_visible = (
                warped_valid
                & (warped > 0.25)
                & (warp_photo_error <= self.temporal_photometric_threshold)
            )
            temporal_supported = temporal_visible & (
                temporal_difference <= temporal_tolerance
            )
            temporal_score[temporal_visible] = np.clip(
                1.0
                - temporal_difference[temporal_visible]
                / np.maximum(2.0 * temporal_tolerance[temporal_visible], 1e-3),
                0.0,
                1.0,
            )
            # Far estimates are admitted only when an optically corresponding
            # previous inference agrees.  Near depth may recover from a
            # marginal LR/photo rejection, but only with strong local and
            # temporal support from two independent model frames.
            valid &= ~(
                (depth >= self.temporal_minimum_depth)
                & temporal_visible
                & ~temporal_supported
            )
            temporal_recovered = (
                base_valid
                & ~valid
                & (depth < self.temporal_minimum_depth)
                & temporal_supported
                & (local_score >= 0.75)
            )
            if self.photometric_threshold > 0.0:
                temporal_recovered &= (
                    photo_difference <= self.photometric_threshold * 1.5
                )
            valid |= temporal_recovered
            fuse = valid & temporal_supported
            disparity[fuse] = 0.80 * disparity[fuse] + 0.20 * warped[fuse]
        filter_stats["temporal"] = float(np.count_nonzero(valid)) / valid.size
        filter_stats["recovered"] = (
            float(np.count_nonzero(temporal_recovered)) / valid.size
        )

        self.previous_temporal_gray = current_gray.copy()
        self.previous_temporal_disparity = raw_disparity.copy()
        with np.errstate(divide="ignore", invalid="ignore"):
            depth = self.model_fx * self.baseline / disparity
        if self.far_artifact_depth > 0.0 and self.far_support_size > 1:
            support = cv2.boxFilter(
                valid.astype(np.uint8), cv2.CV_16U,
                (self.far_support_size, self.far_support_size), normalize=False
            )
            valid &= (depth < self.far_artifact_depth) | (support >= self.far_minimum_support)
        filter_stats["far"] = float(np.count_nonzero(valid)) / valid.size
        confidence, low_texture_risk, reflection_risk = _combine_depth_confidence(
            lr_score,
            photo_score,
            local_score,
            temporal_score,
            temporal_visible,
            texture_score,
            reflection_score,
            self.low_texture_score_threshold,
            self.reflection_score_threshold,
        )
        valid &= ~(low_texture_risk | reflection_risk)
        filter_stats["radiometric"] = float(np.count_nonzero(valid)) / valid.size
        # Saturated or effectively black source pixels have no measurable
        # stereo texture, regardless of how plausible a network value looks.
        source_gray = cv2.cvtColor(
            cv2.resize(
                output_guide,
                (self.model_width, self.model_height),
                interpolation=cv2.INTER_AREA,
            ),
            cv2.COLOR_BGR2GRAY,
        )
        radiometric_valid = (source_gray > 1) & (source_gray < 254)
        valid &= radiometric_valid
        # Keep hard geometry/radiometric vetoes in temporal history, but do
        # not erase a geometrically usable observation merely because its
        # scalar confidence landed just below this frame's output threshold.
        # Otherwise one marginal frame permanently removes the temporal
        # evidence needed to recover the following frame.
        temporal_history_valid = valid.copy()
        if self.model_family == "fast_foundation_stereo":
            # A transient reverse-pass dropout must not permanently erase the
            # next frame's temporal evidence. FFS therefore keeps the
            # pre-LR geometric range/rectification support in history, while
            # retaining radiometric hard vetoes and a weak local-consistency
            # floor. This history is evidence only: current output still has
            # to pass LR and scalar confidence below. The default CREStereo
            # history remains byte-for-byte compatible with confidence_v2.
            temporal_history_valid = (
                base_valid
                & radiometric_valid
                & ~(low_texture_risk | reflection_risk)
                & (local_score >= 0.50)
            )
        valid &= confidence >= self.minimum_depth_confidence
        filter_stats["confidence"] = float(np.count_nonzero(valid)) / valid.size
        self.previous_temporal_valid = temporal_history_valid
        confidence[~valid] = 0.0
        depth[~valid] = 0.0
        valid_ratio = float(np.count_nonzero(valid)) / valid.size
        depth_mm_model = np.clip(depth * 1000.0, 0.0, 65535.0).astype(np.uint16)
        ground_corrected_model = np.zeros(depth_mm_model.shape, dtype=bool)
        if self.ground_plane_prior_enabled:
            # Apply the geometric constraint on the learned model's native
            # calibrated content grid. The
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
        confidence_output = cv2.resize(
            confidence,
            (self.output_width, self.output_height),
            interpolation=cv2.INTER_NEAREST,
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
        confidence_output[depth_mm == 0] = 0.0
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
        raw_disparity_output = cv2.resize(
            raw_disparity,
            (self.output_width, self.output_height),
            interpolation=cv2.INTER_LINEAR,
        ) * (self.output_width / self.model_width)
        confidence_u8 = np.clip(
            np.rint(confidence_output * 255.0), 0.0, 255.0
        ).astype(np.uint8)
        validity_u8 = np.where(final_valid, 255, 0).astype(np.uint8)
        return (
            depth_mm,
            disparity_output.astype(np.float32),
            raw_disparity_output.astype(np.float32),
            confidence_u8,
            validity_u8,
            valid_ratio,
        )

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
                (
                    depth_mm,
                    disparity,
                    raw_disparity,
                    confidence,
                    validity,
                    valid_ratio,
                ) = self._filter_depth(
                    flow, reverse_disparity, left_model, right_model, left_output
                )
                postprocess_ms = (time.monotonic() - postprocess_started) * 1000.0
                publish_started = time.monotonic()
                self._publish(
                    input_header,
                    left_output,
                    depth_mm,
                    disparity,
                    raw_disparity,
                    confidence,
                    validity,
                )
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
                        f"{self.model_display_name}: {fps:.2f} Hz, inference "
                        f"{self.inference_sum_ms / self.log_every_n:.1f} ms, total "
                        f"{self.processing_sum_ms / self.log_every_n:.1f} ms, "
                        f"pre/post {self.preprocess_sum_ms / self.log_every_n:.1f}/"
                        f"{self.postprocess_sum_ms / self.log_every_n:.1f} ms, "
                        f"publish {self.publish_sum_ms / self.log_every_n:.1f} ms, "
                        f"valid {valid_ratio * 100.0:.1f}%, dropped "
                        f"{self.dropped}/{self.preprocessed_dropped}, filter "
                        f"range/rect/vflow/lr/photo/local/temporal/far/ground/out "
                        f"{stages.get('range', 0.0) * 100.0:.1f}/"
                        f"{stages.get('rect', 0.0) * 100.0:.1f}/"
                        f"{stages.get('vflow', 0.0) * 100.0:.1f}/"
                        f"{stages.get('lr', 0.0) * 100.0:.1f}/"
                        f"{stages.get('photo', 0.0) * 100.0:.1f}/"
                        f"{stages.get('local', 0.0) * 100.0:.1f}/"
                        f"{stages.get('temporal', 0.0) * 100.0:.1f}/"
                        f"{stages.get('far', 0.0) * 100.0:.1f}/"
                        f"{stages.get('ground', 0.0) * 100.0:.1f}/"
                        f"{stages.get('output', 0.0) * 100.0:.1f}%, "
                        f"stereo affine {self.last_photometric_gain:.3f}/"
                        f"{self.last_photometric_offset:+.1f}, input luma/gamma "
                        f"{self.last_input_luma:.1f}/{self.last_low_light_gamma:.3f}, "
                        f"temporal recovery "
                        f"{stages.get('recovered', 0.0) * 100.0:.2f}%"
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
                self.get_logger().error(
                    f"{self.model_display_name} frame failed: {error}"
                )

    def _publish(
        self,
        input_header: Header,
        color: np.ndarray,
        depth_mm: np.ndarray,
        disparity: np.ndarray,
        raw_disparity: np.ndarray,
        confidence: np.ndarray,
        validity: np.ndarray,
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
        if self.raw_disparity_publisher.get_subscription_count():
            self.raw_disparity_publisher.publish(
                _image_message(header, "32FC1", raw_disparity)
            )
        if self.confidence_publisher.get_subscription_count():
            self.confidence_publisher.publish(
                _image_message(header, "mono8", confidence)
            )
        if self.validity_publisher.get_subscription_count():
            self.validity_publisher.publish(
                _image_message(header, "mono8", validity)
            )
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
        if getattr(self, "direct_engine", None) is not None:
            self.direct_engine.close()
            self.direct_engine = None
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
