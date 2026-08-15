"""ROS 2 node publishing learned odometry and RTAB-Map RGB-D features."""

from __future__ import annotations

from array import array
from collections import deque
import copy
import math
import threading
import time
from typing import Any

import cv2
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from rtabmap_msgs.msg import KeyPoint, Point3f, RGBDImage
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from .feature_backend import SuperPointLightGlueBackend
from .geometry import invert_transform
from .rtab_codec import compress_descriptor_matrix
from .rate_limit import processing_is_due, publication_is_due
from .tracker import TrackerConfig, TrackingResult, VisualOdometryTracker


def _stamp_seconds(message: Any) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def _quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _transform_matrix(message: TransformStamped) -> np.ndarray:
    transform = message.transform
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = _quaternion_matrix(
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    )
    matrix[:3, 3] = (
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    )
    return matrix


def _matrix_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.asarray(rotation, dtype=np.float64)
    quaternion = np.empty(4, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion[:] = (
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion[:] = (
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
            )
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion[:] = (
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
            )
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion[:] = (
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            )
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


def _camera_intrinsics(message: CameraInfo) -> np.ndarray:
    intrinsics = np.array(
        [[message.k[0], 0.0, message.k[2]], [0.0, message.k[4], message.k[5]], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    if intrinsics[0, 0] <= 0.0 or intrinsics[1, 1] <= 0.0:
        raise ValueError("camera focal length must be positive")
    return intrinsics


def _tiered_mapping_depth(
    depth: np.ndarray,
    depth_scale: float,
    minimum_depth: float,
    dense_maximum_depth: float,
    far_maximum_depth: float,
    far_sparse_pixel_step: int,
    medium_maximum_depth: float = 0.0,
    medium_sparse_pixel_step: int = 1,
) -> np.ndarray:
    """Keep dense near depth plus deterministic medium/far lattices."""
    if far_maximum_depth <= 0.0:
        return depth
    if (
        depth_scale <= 0.0
        or minimum_depth < 0.0
        or dense_maximum_depth <= minimum_depth
        or far_maximum_depth <= dense_maximum_depth
        or far_sparse_pixel_step < 2
        or (
            medium_maximum_depth > 0.0
            and (
                medium_maximum_depth <= dense_maximum_depth
                or medium_maximum_depth >= far_maximum_depth
                or medium_sparse_pixel_step < 2
            )
        )
    ):
        raise ValueError("invalid tiered mapping depth configuration")
    metric = np.asarray(depth, dtype=np.float32) * depth_scale
    near = (
        np.isfinite(metric)
        & (metric >= minimum_depth)
        & (metric <= dense_maximum_depth)
    )
    # A sliced boolean lattice avoids two full-frame int64 coordinate arrays.
    # This helper runs on the NX mapping path, so keep transient memory small.
    far_lattice = np.zeros(metric.shape, dtype=bool)
    far_lattice[::far_sparse_pixel_step, ::far_sparse_pixel_step] = True
    if medium_maximum_depth > 0.0:
        medium_lattice = np.zeros(metric.shape, dtype=bool)
        medium_lattice[::medium_sparse_pixel_step, ::medium_sparse_pixel_step] = True
        medium = (
            np.isfinite(metric)
            & (metric > dense_maximum_depth)
            & (metric <= medium_maximum_depth)
            & medium_lattice
        )
        far_start = medium_maximum_depth
    else:
        medium = np.zeros(metric.shape, dtype=bool)
        far_start = dense_maximum_depth
    far = (
        np.isfinite(metric)
        & (metric > far_start)
        & (metric <= far_maximum_depth)
        & far_lattice
    )
    output = np.zeros_like(depth)
    keep = near | medium | far
    output[keep] = depth[keep]
    return output


def _apply_gravity_ground_plane_prior(
    depth: np.ndarray,
    depth_scale: float,
    intrinsics: np.ndarray,
    world_from_camera_rotation: np.ndarray | None,
    camera_height: float,
    below_ground_tolerance: float,
    minimum_depth: float,
    maximum_depth: float,
    surface_tolerance: float = 0.0,
    maximum_correction: float = 0.0,
    minimum_up_alignment: float = 0.0,
    minimum_row_ratio: float = 0.0,
    reject_unverified_below_plane: bool = False,
    repair_all_below_plane: bool = False,
) -> tuple[np.ndarray, np.ndarray]:
    """Move floor-like and impossible returns to the gravity-aligned floor.

    The synchronized H30 attitude supplies gravity in the optical frame, so
    this remains valid while a legged base pitches or rolls. Existing returns
    close to the measured plane are snapped to it, and physically impossible
    returns slightly below it are repaired. Objects higher than
    ``surface_tolerance`` and unknown pixels are preserved. A positive
    ``maximum_correction`` prevents a gross stereo error from being stretched
    onto the floor across an arbitrarily long ray. A positive
    ``minimum_up_alignment`` additionally requires the local 3-D surface
    normal to agree with gravity, preventing walls and obstacle faces from
    being classified as floor.
    """
    corrected_mask = np.zeros(np.asarray(depth).shape, dtype=bool)
    if world_from_camera_rotation is None or camera_height <= 0.0:
        return depth, corrected_mask
    if (
        depth_scale <= 0.0
        or minimum_depth < 0.0
        or maximum_depth <= minimum_depth
        or below_ground_tolerance < 0.0
        or surface_tolerance < 0.0
        or maximum_correction < 0.0
        or not 0.0 <= minimum_up_alignment <= 1.0
        or not 0.0 <= minimum_row_ratio < 1.0
    ):
        raise ValueError("invalid gravity ground-plane depth configuration")
    image = np.asarray(depth)
    if image.ndim != 2 or image.size == 0:
        raise ValueError("gravity ground-plane prior requires a non-empty depth image")
    matrix = np.asarray(intrinsics, dtype=np.float64)
    rotation = np.asarray(world_from_camera_rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or rotation.shape != (3, 3):
        raise ValueError("gravity ground-plane prior requires 3x3 matrices")
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(rotation)):
        raise ValueError("gravity ground-plane matrices must be finite")
    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    cx, cy = float(matrix[0, 2]), float(matrix[1, 2])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("gravity ground-plane prior requires positive focal lengths")

    # world_from_camera maps optical vectors into the gravity-aligned world.
    # Express world-down in the optical frame and intersect each image ray
    # with the plane one measured camera height below the optical center.
    camera_down = rotation.T @ np.array([0.0, 0.0, -1.0], dtype=np.float64)
    rows = (np.arange(image.shape[0], dtype=np.float32)[:, None] - cy) / fy
    columns = (np.arange(image.shape[1], dtype=np.float32)[None, :] - cx) / fx
    denominator = (
        camera_down[0] * columns
        + camera_down[1] * rows
        + camera_down[2]
    )
    metric = image.astype(np.float32) * depth_scale
    down_distance = metric * denominator
    plane_error = down_distance - camera_height
    physically_below_floor = plane_error > below_ground_tolerance
    below_floor = physically_below_floor.copy()
    if maximum_correction > 0.0:
        below_floor &= plane_error <= maximum_correction
    visible_plane = (
        np.isfinite(metric)
        & (image > 0)
        & (denominator > 1e-6)
    )
    corrected_mask = (
        visible_plane
        & (
            below_floor
            | (np.abs(plane_error) <= surface_tolerance)
        )
    )
    expected = np.divide(
        camera_height,
        denominator,
        out=np.zeros_like(denominator, dtype=np.float64),
        where=denominator > 1e-6,
    )
    expected_in_range = (
        (expected >= minimum_depth)
        & (expected <= maximum_depth)
    )
    corrected_mask &= expected_in_range
    eligible_below_floor = visible_plane & physically_below_floor & expected_in_range
    if minimum_row_ratio > 0.0:
        first_row = int(math.floor(image.shape[0] * minimum_row_ratio))
        corrected_mask[:first_row, :] = False
        eligible_below_floor[:first_row, :] = False
    if minimum_up_alignment > 0.0:
        # Central 3-D differences reject depth discontinuities naturally and
        # let smooth, curved CRE floor estimates be recognized by their local
        # upward normal. Unknown neighbors never create a synthetic surface.
        rays = np.stack(
            (
                np.broadcast_to(columns, image.shape),
                np.broadcast_to(rows, image.shape),
                np.ones(image.shape, dtype=np.float32),
            ),
            axis=2,
        )
        points = rays * metric[:, :, None]
        horizontal = np.zeros_like(points)
        vertical = np.zeros_like(points)
        horizontal[:, 1:-1] = points[:, 2:] - points[:, :-2]
        vertical[1:-1, :] = points[2:, :] - points[:-2, :]
        normals = np.cross(horizontal, vertical)
        normal_norm = np.linalg.norm(normals, axis=2)
        camera_up = rotation.T @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
        alignment = np.abs(normals @ camera_up) / np.maximum(normal_norm, 1e-9)
        neighbor_valid = np.zeros(image.shape, dtype=bool)
        neighbor_valid[1:-1, 1:-1] = (
            (image[1:-1, :-2] > 0)
            & (image[1:-1, 2:] > 0)
            & (image[:-2, 1:-1] > 0)
            & (image[2:, 1:-1] > 0)
        )
        corrected_mask &= (
            neighbor_valid
            & np.isfinite(alignment)
            & (alignment >= minimum_up_alignment)
        )
    if repair_all_below_plane:
        # On a ground robot a downward ray cannot pass through the measured
        # support plane. Restore every such physically impossible CRE return
        # even when disparity flicker makes its local normal unusable. Nearer
        # table legs and other real obstacles remain above the plane and are
        # therefore untouched.
        corrected_mask |= eligible_below_floor
    rejected_mask = (
        eligible_below_floor & ~corrected_mask
        if reject_unverified_below_plane
        else np.zeros(image.shape, dtype=bool)
    )
    if not np.any(corrected_mask) and not np.any(rejected_mask):
        return depth, corrected_mask

    result = image.copy()
    scaled = expected / depth_scale
    if np.issubdtype(result.dtype, np.integer):
        limits = np.iinfo(result.dtype)
        scaled = np.clip(np.rint(scaled), limits.min, limits.max)
    result[corrected_mask] = np.broadcast_to(
        scaled.astype(result.dtype), result.shape
    )[corrected_mask]
    # A downward ray cannot see geometry behind the measured support plane.
    # If its local normal is not trustworthy enough to synthesize floor, drop
    # the return instead of preserving a dense curved sheet as an obstacle.
    result[rejected_mask] = 0
    return result, corrected_mask


class VisualOdometryNode(Node):
    """Run learned RGB-D odometry and publish RTAB-compatible local features."""

    def __init__(self) -> None:
        """Load models, create ROS interfaces, and initialize tracking."""
        super().__init__("luxi_visual_frontend")
        defaults = {
            "device": "cuda",
            "resize_max": 800,
            "max_keypoints": 1024,
            "nms_radius": 4,
            "lightglue_depth_confidence": 0.90,
            "lightglue_width_confidence": 0.95,
            "lightglue_mixed_precision": True,
            "cpu_threads": 4,
            "superpoint_backend": "auto",
            "superpoint_engine_directory": "",
            "superpoint_precision": "fp16",
            "superpoint_cuda_graph": True,
            "lightglue_cuda_graph": True,
            "lightglue_cuda_graph_keypoints": 512,
            "lightglue_cuda_graph_layers": 3,
            "color_topic": "/sensors/rgbd/color/image_raw",
            "depth_topic": "/sensors/rgbd/depth/image_raw",
            "camera_info_topic": "/sensors/rgbd/color/camera_info",
            "rgbd_input_topic": "/sensors/rgbd/rgbd_image",
            "odom_topic": "/luxi_visual_frontend/odom",
            "rgbd_features_topic": "/luxi_visual_frontend/rgbd_image",
            "status_topic": "/luxi_visual_frontend/status",
            "diagnostics_topic": "/luxi_visual_frontend/diagnostics",
            "imu_topic": "/sensors/imu/data",
            "motion_hint_topic": "",
            "motion_hint_timeout": 0.5,
            "motion_hint_stationary_linear_speed": 0.01,
            "motion_hint_stationary_angular_speed": 0.02,
            "odom_frame": "odom",
            "base_frame": "base_link",
            "use_imu_rotation": True,
            "camera_to_imu_time_offset": 0.0,
            "maximum_imu_time_difference": 0.03,
            "target_rate": 5.0,
            "upstream_rate_limited": False,
            "sensor_poll_rate_multiplier": 2.0,
            "rgbd_features_rate": 2.0,
            "maximum_sensor_time_difference": 0.05,
            "transform_timeout": 0.5,
            "depth_scale_16u": 0.001,
            "publish_tf": True,
            "minimum_keypoints": 80,
            "minimum_matches": 50,
            "minimum_depth_matches": 30,
            "minimum_inliers": 25,
            "minimum_inlier_ratio": 0.25,
            "minimum_grid_coverage": 0.25,
            "maximum_reprojection_rmse": 3.0,
            "ransac_reprojection_error": 4.0,
            "ransac_iterations": 1000,
            "ransac_confidence": 0.999,
            "minimum_depth": 0.2,
            "maximum_depth": 6.0,
            "depth_sampling_radius": 0,
            "depth_sampling_minimum_valid": 1,
            "use_depth_translation_refinement": False,
            "keyframe_min_translation": 0.10,
            "keyframe_min_rotation_deg": 8.0,
            "keyframe_max_age": 1.0,
            "keyframe_min_inlier_ratio": 0.40,
            "maximum_frame_translation": 0.25,
            "maximum_frame_rotation_deg": 15.0,
            "maximum_frame_angular_rate_deg": 90.0,
            "maximum_consecutive_tracking_failures": 3,
            "minimum_depth_consistency_matches": 20,
            "maximum_depth_consistency_error": 0.08,
            "maximum_imu_rotation_error_deg": 12.0,
            "maximum_imu_gravity_error_deg": 10.0,
            "stationary_maximum_median_pixel_motion": 0.75,
            "stationary_maximum_rotation_deg": 0.30,
            "stationary_hint_maximum_median_pixel_motion": 2.0,
            "stationary_hint_maximum_translation": 0.05,
            "constrain_vertical_translation": False,
            # Disabled by default. Hardware profiles may retain a dense near
            # layer and publish a much sparser far layer to RTAB-Map without
            # exposing the far depth to visual pose estimation.
            "mapping_depth_minimum": 0.0,
            "mapping_depth_dense_maximum": 0.0,
            "mapping_depth_medium_maximum": 0.0,
            "mapping_depth_medium_sparse_pixel_step": 1,
            "mapping_depth_far_maximum": 0.0,
            "mapping_depth_far_sparse_pixel_step": 1,
            # Disabled for generic cameras. CREStereo uses synchronized H30
            # gravity to correct its smooth-floor bias before metric tracking
            # and RTAB insertion without assuming the walking base is level.
            "mapping_ground_prior_enabled": False,
            "mapping_ground_camera_height": 0.0,
            "mapping_ground_below_tolerance": 0.12,
            "mapping_ground_surface_tolerance": 0.0,
            # Zero retains the historical unlimited correction. CRE profiles
            # set a small bound so a bad disparity cannot be projected across
            # the room and turned into a synthetic floor sheet.
            "mapping_ground_maximum_correction": 0.0,
            "mapping_ground_minimum_up_alignment": 0.0,
            "mapping_ground_minimum_row_ratio": 0.0,
            "mapping_ground_reject_unverified_below_plane": False,
            "mapping_ground_repair_all_below_plane": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        parameters = {name: self.get_parameter(name).value for name in defaults}
        if parameters["target_rate"] <= 0.0:
            raise ValueError("target_rate must be positive")
        if parameters["sensor_poll_rate_multiplier"] < 1.0:
            raise ValueError("sensor_poll_rate_multiplier must be at least one")
        if parameters["rgbd_features_rate"] <= 0.0:
            raise ValueError("rgbd_features_rate must be positive")
        if parameters["maximum_imu_time_difference"] <= 0.0:
            raise ValueError("maximum_imu_time_difference must be positive")
        if (
            float(parameters["motion_hint_timeout"]) <= 0.0
            or float(parameters["motion_hint_stationary_linear_speed"]) < 0.0
            or float(parameters["motion_hint_stationary_angular_speed"]) < 0.0
            or float(parameters["stationary_hint_maximum_median_pixel_motion"]) < 0.0
            or float(parameters["stationary_hint_maximum_translation"]) < 0.0
        ):
            raise ValueError("invalid stationary motion-hint parameters")
        if parameters["minimum_depth_consistency_matches"] < 3:
            raise ValueError("minimum_depth_consistency_matches must be at least 3")
        if parameters["maximum_depth_consistency_error"] <= 0.0:
            raise ValueError("maximum_depth_consistency_error must be positive")
        if parameters["depth_sampling_radius"] < 0:
            raise ValueError("depth_sampling_radius must not be negative")
        if parameters["depth_sampling_minimum_valid"] < 1:
            raise ValueError("depth_sampling_minimum_valid must be positive")
        if float(parameters["mapping_depth_far_maximum"]) > 0.0:
            if (
                float(parameters["mapping_depth_dense_maximum"])
                <= float(parameters["mapping_depth_minimum"])
                or float(parameters["mapping_depth_far_maximum"])
                <= float(parameters["mapping_depth_dense_maximum"])
                or int(parameters["mapping_depth_far_sparse_pixel_step"]) < 2
                or (
                    float(parameters["mapping_depth_medium_maximum"]) > 0.0
                    and (
                        float(parameters["mapping_depth_medium_maximum"])
                        <= float(parameters["mapping_depth_dense_maximum"])
                        or float(parameters["mapping_depth_medium_maximum"])
                        >= float(parameters["mapping_depth_far_maximum"])
                        or int(parameters["mapping_depth_medium_sparse_pixel_step"]) < 2
                    )
                )
            ):
                raise ValueError("invalid tiered mapping depth parameters")
        if bool(parameters["mapping_ground_prior_enabled"]):
            if (
                float(parameters["mapping_ground_camera_height"]) <= 0.0
                or float(parameters["mapping_ground_below_tolerance"]) < 0.0
                or float(parameters["mapping_ground_surface_tolerance"]) < 0.0
                or float(parameters["mapping_ground_maximum_correction"]) < 0.0
                or not 0.0 <= float(
                    parameters["mapping_ground_minimum_up_alignment"]
                ) <= 1.0
                or not 0.0 <= float(
                    parameters["mapping_ground_minimum_row_ratio"]
                ) < 1.0
                or float(parameters["mapping_depth_far_maximum"]) <= 0.0
            ):
                raise ValueError("invalid mapping ground-plane parameters")
        self.get_logger().info("Loading SuperPoint and LightGlue models")
        backend = SuperPointLightGlueBackend(
            str(parameters["device"]),
            int(parameters["resize_max"]),
            int(parameters["max_keypoints"]),
            int(parameters["nms_radius"]),
            float(parameters["lightglue_depth_confidence"]),
            float(parameters["lightglue_width_confidence"]),
            int(parameters["cpu_threads"]),
            str(parameters["superpoint_backend"]),
            str(parameters["superpoint_engine_directory"]),
            str(parameters["superpoint_precision"]),
            bool(parameters["lightglue_mixed_precision"]),
            bool(parameters["superpoint_cuda_graph"]),
            bool(parameters["lightglue_cuda_graph"]),
            int(parameters["lightglue_cuda_graph_keypoints"]),
            int(parameters["lightglue_cuda_graph_layers"]),
        )
        config = TrackerConfig(
            minimum_keypoints=int(parameters["minimum_keypoints"]),
            minimum_matches=int(parameters["minimum_matches"]),
            minimum_depth_matches=int(parameters["minimum_depth_matches"]),
            minimum_inliers=int(parameters["minimum_inliers"]),
            minimum_inlier_ratio=float(parameters["minimum_inlier_ratio"]),
            minimum_grid_coverage=float(parameters["minimum_grid_coverage"]),
            maximum_reprojection_rmse=float(parameters["maximum_reprojection_rmse"]),
            ransac_reprojection_error=float(parameters["ransac_reprojection_error"]),
            ransac_iterations=int(parameters["ransac_iterations"]),
            ransac_confidence=float(parameters["ransac_confidence"]),
            minimum_depth=float(parameters["minimum_depth"]),
            maximum_depth=float(parameters["maximum_depth"]),
            depth_sampling_radius=int(parameters["depth_sampling_radius"]),
            depth_sampling_minimum_valid=int(
                parameters["depth_sampling_minimum_valid"]
            ),
            use_depth_translation_refinement=bool(
                parameters["use_depth_translation_refinement"]
            ),
            keyframe_min_translation=float(parameters["keyframe_min_translation"]),
            keyframe_min_rotation=math.radians(float(parameters["keyframe_min_rotation_deg"])),
            keyframe_max_age=float(parameters["keyframe_max_age"]),
            keyframe_min_inlier_ratio=float(parameters["keyframe_min_inlier_ratio"]),
            maximum_frame_translation=float(parameters["maximum_frame_translation"]),
            maximum_frame_rotation=math.radians(float(parameters["maximum_frame_rotation_deg"])),
            maximum_frame_angular_rate=math.radians(
                float(parameters["maximum_frame_angular_rate_deg"])
            ),
            maximum_consecutive_tracking_failures=int(
                parameters["maximum_consecutive_tracking_failures"]
            ),
            minimum_depth_consistency_matches=int(
                parameters["minimum_depth_consistency_matches"]
            ),
            maximum_depth_consistency_error=float(
                parameters["maximum_depth_consistency_error"]
            ),
            maximum_imu_rotation_error=math.radians(
                float(parameters["maximum_imu_rotation_error_deg"])
            ),
            maximum_imu_gravity_error=math.radians(
                float(parameters["maximum_imu_gravity_error_deg"])
            ),
            stationary_maximum_median_pixel_motion=float(
                parameters["stationary_maximum_median_pixel_motion"]
            ),
            stationary_maximum_rotation=math.radians(
                float(parameters["stationary_maximum_rotation_deg"])
            ),
            stationary_hint_maximum_median_pixel_motion=float(
                parameters["stationary_hint_maximum_median_pixel_motion"]
            ),
            stationary_hint_maximum_translation=float(
                parameters["stationary_hint_maximum_translation"]
            ),
            constrain_vertical_translation=bool(
                parameters["constrain_vertical_translation"]
            ),
        )
        self.parameters = parameters
        self.tracker = VisualOdometryTracker(backend, config)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, parameters["odom_topic"], 10)
        self.rgbd_publisher = self.create_publisher(
            RGBDImage, parameters["rgbd_features_topic"], qos_profile_sensor_data
        )
        self.status_publisher = self.create_publisher(String, parameters["status_topic"], 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, parameters["diagnostics_topic"], 10
        )
        self.create_service(Trigger, "~/reset", self._reset_callback)
        self.lock = threading.Lock()
        # Keep high-rate IMU buffering schedulable while RGB-D inference is
        # running. The default callback group remains mutually exclusive for
        # tracker access; only the short, lock-protected IMU callback is split.
        self.imu_callback_group = MutuallyExclusiveCallbackGroup()
        self.motion_callback_group = MutuallyExclusiveCallbackGroup()
        self.sensor_callback_group = MutuallyExclusiveCallbackGroup()
        self.latest_color: Image | None = None
        self.latest_depth: Image | None = None
        self.latest_camera_info: CameraInfo | None = None
        self.processed_stamp = -1.0
        self.processing = False
        self.base_from_camera: np.ndarray | None = None
        self.last_base_pose: np.ndarray | None = None
        self.last_pose_stamp: float | None = None
        self.last_rgbd_features_stamp: float | None = None
        self.imu_samples: deque[tuple[float, str, np.ndarray]] = deque(maxlen=400)
        self.camera_from_imu_rotations: dict[tuple[str, str], np.ndarray] = {}
        self.latest_motion_hint: tuple[float, float, float] | None = None
        # Inference is intentionally slower than the 10 Hz sensor stream.  A
        # depth-one best-effort input prevents stale full-resolution RGB-D
        # packets from queueing while the current frame is being processed.
        latest_sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        if bool(parameters["use_imu_rotation"]):
            imu_history_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=100,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                Imu,
                parameters["imu_topic"],
                self._imu_callback,
                imu_history_qos,
                callback_group=self.imu_callback_group,
            )
        if str(parameters["motion_hint_topic"]):
            self.create_subscription(
                Twist,
                str(parameters["motion_hint_topic"]),
                self._motion_hint_callback,
                10,
                callback_group=self.motion_callback_group,
            )
        atomic_rgbd_input = bool(parameters["rgbd_input_topic"])
        if atomic_rgbd_input:
            self.create_subscription(
                RGBDImage,
                parameters["rgbd_input_topic"],
                self._rgbd_callback,
                latest_sensor_qos,
                callback_group=self.sensor_callback_group,
            )
        else:
            self.create_subscription(
                Image, parameters["color_topic"], self._color_callback,
                latest_sensor_qos, callback_group=self.sensor_callback_group,
            )
            self.create_subscription(
                Image, parameters["depth_topic"], self._depth_callback,
                latest_sensor_qos, callback_group=self.sensor_callback_group,
            )
            self.create_subscription(
                CameraInfo,
                parameters["camera_info_topic"],
                self._camera_info_callback,
                latest_sensor_qos,
                callback_group=self.sensor_callback_group,
            )
        # Keep the DDS callback short even for atomic RGB-D. Running the GPU
        # tracker inside that callback made the depth-one input reader drop
        # frames while it was busy. A 2x polling timer observes the newest
        # packet promptly; the timestamp limiter below still bounds actual
        # tracking to target_rate and never builds a stale queue.
        poll_rate = float(parameters["target_rate"]) * (
            float(parameters["sensor_poll_rate_multiplier"])
            if atomic_rgbd_input else 1.0
        )
        self.timer = self.create_timer(1.0 / poll_rate, self._process_latest)
        self._publish_status("WAITING_FOR_SENSOR_DATA")
        self.get_logger().info(f"Learned frontend inference device: {backend.device}")
        if float(parameters["mapping_depth_far_maximum"]) > 0.0:
            medium_maximum = float(parameters["mapping_depth_medium_maximum"])
            medium_description = ""
            far_start = float(parameters["mapping_depth_dense_maximum"])
            if medium_maximum > 0.0:
                medium_step = int(parameters["mapping_depth_medium_sparse_pixel_step"])
                medium_description = (
                    f"{far_start:.1f}-{medium_maximum:.1f} m "
                    f"1/{medium_step}x{medium_step} sparse, "
                )
                far_start = medium_maximum
            self.get_logger().info(
                "RTAB depth tiers: "
                f"{float(parameters['mapping_depth_minimum']):.1f}-"
                f"{float(parameters['mapping_depth_dense_maximum']):.1f} m dense, "
                f"{medium_description}{far_start:.1f}-"
                f"{float(parameters['mapping_depth_far_maximum']):.1f} m 1/"
                f"{int(parameters['mapping_depth_far_sparse_pixel_step'])}x"
                f"{int(parameters['mapping_depth_far_sparse_pixel_step'])} sparse"
            )
        if bool(parameters["mapping_ground_prior_enabled"]):
            self.get_logger().info(
                "RTAB-only ground prior: H30 gravity, "
                f"camera_height={float(parameters['mapping_ground_camera_height']):.3f} m, "
                f"below_tolerance={float(parameters['mapping_ground_below_tolerance']):.3f} m, "
                f"surface_tolerance="
                f"{float(parameters['mapping_ground_surface_tolerance']):.3f} m, "
                f"maximum_correction="
                f"{float(parameters['mapping_ground_maximum_correction']):.3f} m, "
                f"minimum_up_alignment="
                f"{float(parameters['mapping_ground_minimum_up_alignment']):.2f}, "
                f"minimum_row_ratio="
                f"{float(parameters['mapping_ground_minimum_row_ratio']):.2f}, "
                f"reject_unverified="
                f"{bool(parameters['mapping_ground_reject_unverified_below_plane'])}, "
                f"repair_all_below="
                f"{bool(parameters['mapping_ground_repair_all_below_plane'])}"
            )
        if str(parameters["motion_hint_topic"]):
            self.get_logger().info(
                "Stationary pose gate uses motion hint "
                f"{parameters['motion_hint_topic']} with "
                f"{float(parameters['motion_hint_timeout']):.2f} s timeout"
            )

    def _color_callback(self, message: Image) -> None:
        with self.lock:
            self.latest_color = message

    def _depth_callback(self, message: Image) -> None:
        with self.lock:
            self.latest_depth = message

    def _camera_info_callback(self, message: CameraInfo) -> None:
        with self.lock:
            self.latest_camera_info = message

    def _imu_callback(self, message: Imu) -> None:
        """Buffer valid AHRS orientations for camera-time interpolation."""
        if message.orientation_covariance[0] < 0.0:
            return
        try:
            rotation = _quaternion_matrix(
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            )
        except ValueError:
            return
        with self.lock:
            self.imu_samples.append(
                (_stamp_seconds(message), message.header.frame_id, rotation)
            )

    def _motion_hint_callback(self, message: Twist) -> None:
        """Record the D1 command without coupling odometry to robot feedback."""
        linear_speed = math.hypot(float(message.linear.x), float(message.linear.y))
        angular_speed = abs(float(message.angular.z))
        with self.lock:
            self.latest_motion_hint = (
                time.monotonic(), linear_speed, angular_speed
            )

    def _commanded_stationary(self) -> bool:
        """Return true only for a fresh, explicitly zero robot command."""
        with self.lock:
            sample = self.latest_motion_hint
        if sample is None:
            return False
        received, linear_speed, angular_speed = sample
        return (
            time.monotonic() - received
            <= float(self.parameters["motion_hint_timeout"])
            and linear_speed
            <= float(self.parameters["motion_hint_stationary_linear_speed"])
            and angular_speed
            <= float(self.parameters["motion_hint_stationary_angular_speed"])
        )

    def _rgbd_callback(self, message: RGBDImage) -> None:
        """Commit one adapter-normalized RGB-D packet atomically."""
        with self.lock:
            self.latest_color = message.rgb
            self.latest_depth = message.depth
            self.latest_camera_info = message.rgb_camera_info

    def _snapshot(self) -> tuple[Image, Image, CameraInfo] | None:
        with self.lock:
            if self.latest_color is None or self.latest_depth is None or self.latest_camera_info is None:
                return None
            return self.latest_color, self.latest_depth, self.latest_camera_info

    def _reset_callback(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.tracker.reset()
        self.last_base_pose = None
        self.last_pose_stamp = None
        self.last_rgbd_features_stamp = None
        response.success = True
        response.message = "visual frontend tracking state reset"
        self._publish_status("WAITING_FOR_SENSOR_DATA")
        return response

    def _publish_status(self, state: str, reason: str = "") -> None:
        message = String()
        message.data = state if not reason else f"{state}:{reason}"
        self.status_publisher.publish(message)

    def _camera_transform(self, camera_frame: str) -> np.ndarray:
        if self.base_from_camera is None:
            transform = self.tf_buffer.lookup_transform(
                str(self.parameters["base_frame"]),
                camera_frame,
                Time(),
                timeout=Duration(seconds=float(self.parameters["transform_timeout"])),
            )
            self.base_from_camera = _transform_matrix(transform)
        return self.base_from_camera

    def _world_from_camera_rotation(
        self, stamp: float, camera_frame: str
    ) -> tuple[np.ndarray | None, float | None]:
        """Return the nearest calibrated IMU camera orientation and time error."""
        if not bool(self.parameters["use_imu_rotation"]):
            return None, None
        target_stamp = stamp + float(self.parameters["camera_to_imu_time_offset"])
        with self.lock:
            sample = min(
                self.imu_samples,
                key=lambda item: abs(item[0] - target_stamp),
                default=None,
            )
        if sample is None:
            return None, None
        time_error = abs(sample[0] - target_stamp)
        if time_error > float(self.parameters["maximum_imu_time_difference"]):
            return None, time_error
        _, imu_frame, world_from_imu = sample
        key = (camera_frame, imu_frame)
        camera_from_imu = self.camera_from_imu_rotations.get(key)
        if camera_from_imu is None:
            transform = self.tf_buffer.lookup_transform(
                camera_frame,
                imu_frame,
                Time(),
                timeout=Duration(seconds=float(self.parameters["transform_timeout"])),
            )
            camera_from_imu = _transform_matrix(transform)[:3, :3]
            self.camera_from_imu_rotations[key] = camera_from_imu
        return world_from_imu @ camera_from_imu.T, time_error

    def _publish_diagnostics(self, result: TrackingResult) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "luxi_visual_frontend/tracking"
        status.hardware_id = "learned_rgbd_frontend"
        status.level = DiagnosticStatus.OK if result.accepted else DiagnosticStatus.WARN
        status.message = result.reason
        values = {
            "state": result.state,
            "superpoint_backend": self.tracker.backend.active_superpoint_backend,
            "lightglue_backend": self.tracker.backend.active_lightglue_backend,
            "extract_seconds": round(self.tracker.backend.last_extract_seconds, 5),
            "match_seconds": round(self.tracker.backend.last_match_seconds, 5),
            "match_layers": self.tracker.backend.last_match_layers,
            "keypoints": result.keypoint_count,
            "matches": result.match_count,
            "depth_matches": result.depth_match_count,
            "inliers": result.inlier_count,
            "inlier_ratio": round(result.inlier_ratio, 5),
            "grid_coverage": round(result.grid_coverage, 5),
            "reprojection_rmse": result.reprojection_rmse,
            "elapsed_seconds": round(result.elapsed_seconds, 5),
            "keyframe_updated": result.keyframe_updated,
            "pose_source": result.pose_source,
            "imu_rotation_error_deg": None
            if result.imu_rotation_error is None
            else round(math.degrees(result.imu_rotation_error), 4),
            "imu_gravity_error_deg": None
            if result.imu_gravity_error is None
            else round(math.degrees(result.imu_gravity_error), 4),
            "depth_consistency_inliers": result.depth_consistency_inliers,
            "median_pixel_motion": None
            if result.median_pixel_motion is None
            else round(result.median_pixel_motion, 4),
            "commanded_stationary": self._commanded_stationary(),
        }
        status.values = [KeyValue(key=name, value=str(value)) for name, value in values.items()]
        message.status = [status]
        self.diagnostics_publisher.publish(message)

    @staticmethod
    def _set_pose(message: Any, transform: np.ndarray) -> None:
        translation = transform[:3, 3]
        quaternion = _matrix_quaternion(transform[:3, :3])
        message.position.x, message.position.y, message.position.z = map(float, translation)
        (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        ) = quaternion

    def _publish_odometry(self, result: TrackingResult, color: Image) -> np.ndarray:
        assert result.odom_from_camera is not None and self.base_from_camera is not None
        camera_from_base = invert_transform(self.base_from_camera)
        odom_from_base = result.odom_from_camera @ camera_from_base
        stamp = _stamp_seconds(color)
        message = Odometry()
        message.header.stamp = color.header.stamp
        message.header.frame_id = str(self.parameters["odom_frame"])
        message.child_frame_id = str(self.parameters["base_frame"])
        self._set_pose(message.pose.pose, odom_from_base)
        position_variance = max(0.0025, min(1.0, result.reprojection_rmse or 0.05) ** 2 * 0.01)
        angle_variance = max(0.005, position_variance * 2.0)
        message.pose.covariance[0] = position_variance
        message.pose.covariance[7] = position_variance
        message.pose.covariance[14] = position_variance * 2.0
        message.pose.covariance[21] = angle_variance
        message.pose.covariance[28] = angle_variance
        message.pose.covariance[35] = angle_variance
        if self.last_base_pose is not None and self.last_pose_stamp is not None and stamp > self.last_pose_stamp:
            delta = invert_transform(self.last_base_pose) @ odom_from_base
            elapsed = stamp - self.last_pose_stamp
            message.twist.twist.linear.x = float(delta[0, 3] / elapsed)
            message.twist.twist.linear.y = float(delta[1, 3] / elapsed)
            message.twist.twist.linear.z = float(delta[2, 3] / elapsed)
            rotation_vector, _ = cv2.Rodrigues(delta[:3, :3])
            message.twist.twist.angular.x = float(rotation_vector[0, 0] / elapsed)
            message.twist.twist.angular.y = float(rotation_vector[1, 0] / elapsed)
            message.twist.twist.angular.z = float(rotation_vector[2, 0] / elapsed)
        message.twist.covariance = message.pose.covariance
        if bool(self.parameters["publish_tf"]):
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = message.child_frame_id
            transform.transform.translation.x = message.pose.pose.position.x
            transform.transform.translation.y = message.pose.pose.position.y
            transform.transform.translation.z = message.pose.pose.position.z
            transform.transform.rotation = message.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)
        # Publish TF first. RTAB-Map may consume odometry and RGBD immediately
        # and query this exact sensor stamp before a later TF message arrives.
        self.odom_publisher.publish(message)
        self.last_base_pose = odom_from_base
        self.last_pose_stamp = stamp
        return odom_from_base

    def _publish_rgbd_features(
        self,
        result: TrackingResult,
        color: Image,
        depth: Image,
        depth_image: np.ndarray,
        depth_scale: float,
        camera_info: CameraInfo,
    ) -> None:
        message = RGBDImage()
        message.header.stamp = color.header.stamp
        message.header.frame_id = color.header.frame_id
        normalized_camera_info = copy.deepcopy(camera_info)
        normalized_camera_info.header = color.header
        message.rgb_camera_info = normalized_camera_info
        message.depth_camera_info = normalized_camera_info
        message.rgb = color
        mapping_depth = _tiered_mapping_depth(
            depth_image,
            depth_scale,
            float(self.parameters["mapping_depth_minimum"]),
            float(self.parameters["mapping_depth_dense_maximum"]),
            float(self.parameters["mapping_depth_far_maximum"]),
            int(self.parameters["mapping_depth_far_sparse_pixel_step"]),
            float(self.parameters["mapping_depth_medium_maximum"]),
            int(self.parameters["mapping_depth_medium_sparse_pixel_step"]),
        )
        if mapping_depth is depth_image:
            message.depth = depth
        else:
            mapping_depth_message = self.bridge.cv2_to_imgmsg(
                mapping_depth, encoding=depth.encoding
            )
            mapping_depth_message.header = depth.header
            message.depth = mapping_depth_message
        keypoints = []
        points = []
        for index, pixel in enumerate(result.features.keypoints):
            keypoint = KeyPoint()
            keypoint.pt.x = float(pixel[0])
            keypoint.pt.y = float(pixel[1])
            keypoint.size = 1.0
            keypoint.angle = -1.0
            keypoint.response = float(result.features.scores[index])
            keypoint.octave = 0
            keypoint.class_id = index
            keypoints.append(keypoint)
            point = Point3f()
            point.x, point.y, point.z = map(float, result.feature_points3d[index])
            points.append(point)
        message.key_points = keypoints
        message.points = points
        payload = compress_descriptor_matrix(result.features.descriptor_array())
        message.descriptors = array("B", payload)
        self.rgbd_publisher.publish(message)

    def _process_latest(self) -> None:
        if self.processing:
            return
        snapshot = self._snapshot()
        if snapshot is None:
            self._publish_status("WAITING_FOR_SENSOR_DATA")
            return
        color, depth, camera_info = snapshot
        stamp = _stamp_seconds(color)
        if stamp <= self.processed_stamp:
            return
        if (
            not bool(self.parameters["upstream_rate_limited"])
            and not processing_is_due(
                self.processed_stamp, stamp, float(self.parameters["target_rate"])
            )
        ):
            return
        if abs(stamp - _stamp_seconds(depth)) > float(
            self.parameters["maximum_sensor_time_difference"]
        ):
            self._publish_status("DEGRADED", "SENSOR_TIME_MISMATCH")
            return
        if abs(stamp - _stamp_seconds(camera_info)) > float(
            self.parameters["maximum_sensor_time_difference"]
        ):
            self._publish_status("DEGRADED", "CAMERA_INFO_TIME_MISMATCH")
            return
        self.processing = True
        self.processed_stamp = stamp
        self._publish_status("PROCESSING")
        try:
            rgb = self.bridge.imgmsg_to_cv2(color, desired_encoding="rgb8")
            depth_image = self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
            if depth_image.dtype == np.uint16:
                depth_scale = float(self.parameters["depth_scale_16u"])
            elif depth_image.dtype == np.float32:
                depth_scale = 1.0
            else:
                raise ValueError(f"unsupported depth dtype: {depth_image.dtype}")
            base_from_camera = self._camera_transform(color.header.frame_id)
            world_from_camera_rotation, imu_time_error = (
                self._world_from_camera_rotation(stamp, color.header.frame_id)
            )
            # PnP must see the actual CRE measurement. Feeding it a synthetic
            # gravity plane made tens of thousands of unrelated keypoints
            # share one fabricated depth and coupled floor repair into pose.
            tracking_depth = np.asarray(depth_image)
            result = self.tracker.process(
                np.asarray(rgb),
                tracking_depth,
                _camera_intrinsics(camera_info),
                depth_scale,
                stamp,
                base_from_camera,
                world_from_camera_rotation,
                base_from_camera,
                stationary_hint=self._commanded_stationary(),
            )
            # The first frame establishes the odometry origin, so it has no
            # relative rotation to constrain. Warn only after a pose exists.
            if (
                world_from_camera_rotation is None
                and bool(self.parameters["use_imu_rotation"])
                and self.last_pose_stamp is not None
            ):
                self.get_logger().warn(
                    "No synchronized IMU orientation for RGB-D stamp "
                    f"{stamp:.6f} (nearest error={imu_time_error})",
                    throttle_duration_sec=2.0,
                )
            self._publish_diagnostics(result)
            self._publish_status(result.state, result.reason)
            if result.accepted:
                self._publish_odometry(result, color)
                if publication_is_due(
                    self.last_rgbd_features_stamp,
                    stamp,
                    float(self.parameters["rgbd_features_rate"]),
                ):
                    mapping_depth = tracking_depth
                    if bool(self.parameters["mapping_ground_prior_enabled"]):
                        mapping_depth, ground_corrected = (
                            _apply_gravity_ground_plane_prior(
                                mapping_depth,
                                depth_scale,
                                _camera_intrinsics(camera_info),
                                world_from_camera_rotation,
                                float(self.parameters["mapping_ground_camera_height"]),
                                float(self.parameters["mapping_ground_below_tolerance"]),
                                float(self.parameters["mapping_depth_minimum"]),
                                float(self.parameters["mapping_depth_far_maximum"]),
                                float(self.parameters["mapping_ground_surface_tolerance"]),
                                float(self.parameters["mapping_ground_maximum_correction"]),
                                float(
                                    self.parameters[
                                        "mapping_ground_minimum_up_alignment"
                                    ]
                                ),
                                float(
                                    self.parameters[
                                        "mapping_ground_minimum_row_ratio"
                                    ]
                                ),
                                bool(
                                    self.parameters[
                                        "mapping_ground_reject_unverified_below_plane"
                                    ]
                                ),
                                bool(
                                    self.parameters[
                                        "mapping_ground_repair_all_below_plane"
                                    ]
                                ),
                            )
                        )
                        corrected_count = int(np.count_nonzero(ground_corrected))
                        if corrected_count > 0:
                            self.get_logger().info(
                                "RTAB-only ground prior corrected "
                                f"{corrected_count} physically valid floor pixels",
                                throttle_duration_sec=5.0,
                            )
                        elif world_from_camera_rotation is None:
                            self.get_logger().warn(
                                "RTAB-only ground prior skipped: no synchronized H30 attitude",
                                throttle_duration_sec=2.0,
                            )
                    self._publish_rgbd_features(
                        result,
                        color,
                        depth,
                        mapping_depth,
                        depth_scale,
                        camera_info,
                    )
                    self.last_rgbd_features_stamp = stamp
        except Exception as error:  # ROS boundary: report and keep the node diagnosable.
            if not self.context.ok():
                return
            self.get_logger().error(f"Visual frontend frame failed: {error}")
            self._publish_status("ERROR", type(error).__name__)
        finally:
            self.processing = False


def main(arguments: list[str] | None = None) -> None:
    """Run the learned visual odometry node until ROS shuts down."""
    rclpy.init(args=arguments)
    node = VisualOdometryNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            executor.shutdown(timeout_sec=2.0)
        except KeyboardInterrupt:
            pass
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
