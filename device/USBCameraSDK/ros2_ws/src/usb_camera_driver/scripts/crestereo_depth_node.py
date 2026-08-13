#!/usr/bin/env python3
"""CREStereo ONNX RGB-D frontend for the calibrated USB stereo camera.

The node intentionally lives beside, rather than inside, the production VPI
frontend.  It keeps only the newest synchronized stereo pair while inference
is busy so RTAB-Map receives fresh frames instead of an ever-growing backlog.
"""

from __future__ import annotations

from array import array
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

        cv2.setNumThreads(max(1, int(self.declare_parameter("opencv_num_threads", 2).value)))
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
        self.left_messages: Dict[int, object] = {}
        self.right_messages: Dict[int, object] = {}
        self.pending_pair: Optional[Tuple[object, object]] = None
        self.stop_event = threading.Event()
        self.frames = 0
        self.dropped = 0
        self.last_point_cloud_time = 0.0
        self.last_log_time = time.monotonic()
        self.inference_sum_ms = 0.0
        self.processing_sum_ms = 0.0
        self.publish_sum_ms = 0.0
        self.next_inference_time = 0.0
        self.worker = threading.Thread(target=self._worker, name="crestereo-inference", daemon=True)
        self.worker.start()
        self.context.on_shutdown(self._request_stop)
        active = ",".join(self.session.get_providers())
        profile = "refined" if self.has_refinement else "single-pass"
        self.get_logger().info(
            f"CREStereo ready: {profile} {self.model_width}x{self.model_height}, "
            f"providers={active}, RGB-D={self.output_width}x{self.output_height}, "
            f"depth={self.minimum_depth:.1f}-{self.maximum_depth:.1f} m, "
            f"cuda_graph={'on' if self.io_binding is not None and self.enable_cuda_graph else 'off'}"
        )

    def _create_session(self, model_path: str, provider: str, cache_path: str):
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
                "enable_cuda_graph": self.enable_cuda_graph,
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
        self.model_fx = float(model_projection_left[0, 0])
        self.baseline = abs(float(p_right[0, 3] / p_right[0, 0]))
        self.left_model_maps = cv2.initUndistortRectifyMap(
            k_left, d_left, r_left, model_projection_left[:, :3],
            (self.model_width, self.model_height), cv2.CV_32FC1
        )
        self.right_model_maps = cv2.initUndistortRectifyMap(
            k_right, d_right, r_right, model_projection_right[:, :3],
            (self.model_width, self.model_height), cv2.CV_32FC1
        )
        self.left_output_maps = cv2.initUndistortRectifyMap(
            k_left, d_left, r_left, self.output_projection[:, :3],
            (self.output_width, self.output_height), cv2.CV_32FC1
        )
        source_valid = np.full((original_height, original_width), 255, dtype=np.uint8)
        self.left_model_valid = cv2.remap(
            source_valid, *self.left_model_maps, cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT, borderValue=0
        ) > 0
        self.right_model_valid = cv2.remap(
            source_valid, *self.right_model_maps, cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT, borderValue=0
        ) > 0

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

    def _filter_depth(
        self, flow: np.ndarray, left: np.ndarray, right: np.ndarray
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
        x_right = np.rint(
            np.arange(self.model_width, dtype=np.float32)[None, :] - disparity
        ).astype(np.int32)
        inside = (x_right >= 0) & (x_right < self.model_width)
        clipped = np.clip(x_right, 0, self.model_width - 1)
        rows = np.arange(self.model_height)[:, None]
        valid &= inside & self.right_model_valid[rows, clipped]
        if flow.shape[0] > 1 and self.maximum_vertical_flow > 0.0:
            valid &= np.abs(flow[1]) <= self.maximum_vertical_flow
        if self.photometric_threshold > 0.0:
            left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY).astype(np.int16)
            right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY).astype(np.int16)
            difference = np.abs(left_gray - right_gray[rows, clipped])
            valid &= difference <= self.photometric_threshold
        if self.far_artifact_depth > 0.0 and self.far_support_size > 1:
            support = cv2.boxFilter(
                valid.astype(np.uint8), cv2.CV_16U,
                (self.far_support_size, self.far_support_size), normalize=False
            )
            valid &= (depth < self.far_artifact_depth) | (support >= self.far_minimum_support)
        depth[~valid] = 0.0
        valid_ratio = float(np.count_nonzero(valid)) / valid.size
        depth_mm_model = np.clip(depth * 1000.0, 0.0, 65535.0).astype(np.uint16)
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
        disparity_output = cv2.resize(
            disparity, (self.output_width, self.output_height), interpolation=cv2.INTER_NEAREST
        ) * (self.output_width / self.model_width)
        disparity_output[depth_mm == 0] = -1.0
        return depth_mm, disparity_output.astype(np.float32), valid_ratio

    def _worker(self) -> None:
        while not self.stop_event.is_set() and rclpy.ok():
            with self.pair_lock:
                while not self.stop_event.is_set():
                    now = time.monotonic()
                    delay = self.next_inference_time - now
                    if self.pending_pair is not None and delay <= 0.0:
                        break
                    # New pairs wake this wait and replace the pending slot;
                    # inference always takes the newest pair at the next slot.
                    self.pair_lock.wait(timeout=max(0.001, min(0.5, delay)) if delay > 0.0 else 0.5)
                pair, self.pending_pair = self.pending_pair, None
            if pair is None:
                continue
            try:
                started = time.monotonic()
                self.next_inference_time = started + 1.0 / self.target_rate
                left_source, right_source = self._decode(pair[0]), self._decode(pair[1])
                if left_source.shape[1::-1] != self.original_size or right_source.shape[1::-1] != self.original_size:
                    raise RuntimeError(
                        f"input size {left_source.shape[1]}x{left_source.shape[0]} does not match "
                        f"calibration {self.original_size[0]}x{self.original_size[1]}"
                    )
                left_model = cv2.remap(
                    left_source, *self.left_model_maps, cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT
                )
                right_model = cv2.remap(
                    right_source, *self.right_model_maps, cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT
                )
                left_output = cv2.remap(
                    left_source, *self.left_output_maps, cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT
                )
                inference_started = time.monotonic()
                flow = self._infer(left_model, right_model)
                inference_ms = (time.monotonic() - inference_started) * 1000.0
                depth_mm, disparity, valid_ratio = self._filter_depth(
                    flow, left_model, right_model
                )
                publish_started = time.monotonic()
                self._publish(pair[0].header, left_output, depth_mm, disparity)
                publish_ms = (time.monotonic() - publish_started) * 1000.0
                processing_ms = (time.monotonic() - started) * 1000.0
                self.frames += 1
                self.inference_sum_ms += inference_ms
                self.processing_sum_ms += processing_ms
                self.publish_sum_ms += publish_ms
                if self.frames % self.log_every_n == 0:
                    now = time.monotonic()
                    fps = self.log_every_n / max(now - self.last_log_time, 1e-6)
                    self.get_logger().info(
                        f"CREStereo: {fps:.2f} Hz, inference "
                        f"{self.inference_sum_ms / self.log_every_n:.1f} ms, total "
                        f"{self.processing_sum_ms / self.log_every_n:.1f} ms, "
                        f"publish {self.publish_sum_ms / self.log_every_n:.1f} ms, "
                        f"valid {valid_ratio * 100.0:.1f}%, dropped {self.dropped}"
                    )
                    self.inference_sum_ms = 0.0
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
                self.pair_lock.notify_all()

    def destroy_node(self) -> bool:
        self._request_stop()
        if hasattr(self, "worker") and self.worker.is_alive():
            # ROS launch may forward SIGINT while Python is joining. Keep
            # waiting for the CUDA call to return so ORT is not torn down
            # underneath its worker thread (which otherwise aborts at exit).
            deadline = time.monotonic() + 3.0
            while self.worker.is_alive() and time.monotonic() < deadline:
                try:
                    self.worker.join(timeout=0.25)
                except KeyboardInterrupt:
                    continue
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
