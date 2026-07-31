"""ROS 2 node publishing learned odometry and RTAB-Map RGB-D features."""

from __future__ import annotations

from array import array
import math
import threading
from typing import Any

import cv2
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rtabmap_msgs.msg import KeyPoint, Point3f, RGBDImage
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from .feature_backend import SuperPointLightGlueBackend
from .geometry import invert_transform
from .rtab_codec import compress_descriptor_matrix
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
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * w), 2.0 * (y * z - x * w)],
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


class VisualOdometryNode(Node):
    """Run learned RGB-D odometry and publish RTAB-compatible local features."""

    def __init__(self) -> None:
        """Load models, create ROS interfaces, and initialize tracking."""
        super().__init__("luxi_visual_frontend")
        defaults = {
            "device": "cuda",
            "resize_max": 800,
            "max_keypoints": 2048,
            "nms_radius": 3,
            "lightglue_depth_confidence": 0.90,
            "lightglue_width_confidence": 0.95,
            "cpu_threads": 4,
            "color_topic": "/sensors/rgbd/color/image_raw",
            "depth_topic": "/sensors/rgbd/depth/image_raw",
            "camera_info_topic": "/sensors/rgbd/color/camera_info",
            "odom_topic": "/luxi_visual_frontend/odom",
            "rgbd_features_topic": "/luxi_visual_frontend/rgbd_image",
            "status_topic": "/luxi_visual_frontend/status",
            "diagnostics_topic": "/luxi_visual_frontend/diagnostics",
            "odom_frame": "odom",
            "base_frame": "base_link",
            "target_rate": 10.0,
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
            "keyframe_min_translation": 0.10,
            "keyframe_min_rotation_deg": 8.0,
            "keyframe_max_age": 1.0,
            "keyframe_min_inlier_ratio": 0.40,
            "maximum_frame_translation": 1.0,
            "maximum_frame_rotation_deg": 60.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        parameters = {name: self.get_parameter(name).value for name in defaults}
        if parameters["target_rate"] <= 0.0:
            raise ValueError("target_rate must be positive")

        self.get_logger().info("Loading SuperPoint and LightGlue models")
        backend = SuperPointLightGlueBackend(
            str(parameters["device"]),
            int(parameters["resize_max"]),
            int(parameters["max_keypoints"]),
            int(parameters["nms_radius"]),
            float(parameters["lightglue_depth_confidence"]),
            float(parameters["lightglue_width_confidence"]),
            int(parameters["cpu_threads"]),
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
            keyframe_min_translation=float(parameters["keyframe_min_translation"]),
            keyframe_min_rotation=math.radians(float(parameters["keyframe_min_rotation_deg"])),
            keyframe_max_age=float(parameters["keyframe_max_age"]),
            keyframe_min_inlier_ratio=float(parameters["keyframe_min_inlier_ratio"]),
            maximum_frame_translation=float(parameters["maximum_frame_translation"]),
            maximum_frame_rotation=math.radians(float(parameters["maximum_frame_rotation_deg"])),
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
        self.create_subscription(
            Image, parameters["color_topic"], self._color_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, parameters["depth_topic"], self._depth_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            parameters["camera_info_topic"],
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.lock = threading.Lock()
        self.latest_color: Image | None = None
        self.latest_depth: Image | None = None
        self.latest_camera_info: CameraInfo | None = None
        self.processed_stamp = -1.0
        self.processing = False
        self.base_from_camera: np.ndarray | None = None
        self.last_base_pose: np.ndarray | None = None
        self.last_pose_stamp: float | None = None
        self.timer = self.create_timer(1.0 / float(parameters["target_rate"]), self._process_latest)
        self._publish_status("WAITING_FOR_SENSOR_DATA")
        self.get_logger().info(f"Learned frontend inference device: {backend.device}")

    def _color_callback(self, message: Image) -> None:
        with self.lock:
            self.latest_color = message

    def _depth_callback(self, message: Image) -> None:
        with self.lock:
            self.latest_depth = message

    def _camera_info_callback(self, message: CameraInfo) -> None:
        with self.lock:
            self.latest_camera_info = message

    def _snapshot(self) -> tuple[Image, Image, CameraInfo] | None:
        with self.lock:
            if self.latest_color is None or self.latest_depth is None or self.latest_camera_info is None:
                return None
            return self.latest_color, self.latest_depth, self.latest_camera_info

    def _reset_callback(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.tracker.reset()
        self.last_base_pose = None
        self.last_pose_stamp = None
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
            "keypoints": result.keypoint_count,
            "matches": result.match_count,
            "depth_matches": result.depth_match_count,
            "inliers": result.inlier_count,
            "inlier_ratio": round(result.inlier_ratio, 5),
            "grid_coverage": round(result.grid_coverage, 5),
            "reprojection_rmse": result.reprojection_rmse,
            "elapsed_seconds": round(result.elapsed_seconds, 5),
            "keyframe_updated": result.keyframe_updated,
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
        self.odom_publisher.publish(message)
        if bool(self.parameters["publish_tf"]):
            transform = TransformStamped()
            transform.header = message.header
            transform.child_frame_id = message.child_frame_id
            transform.transform.translation.x = message.pose.pose.position.x
            transform.transform.translation.y = message.pose.pose.position.y
            transform.transform.translation.z = message.pose.pose.position.z
            transform.transform.rotation = message.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)
        self.last_base_pose = odom_from_base
        self.last_pose_stamp = stamp
        return odom_from_base

    def _publish_rgbd_features(
        self,
        result: TrackingResult,
        color: Image,
        depth: Image,
        camera_info: CameraInfo,
    ) -> None:
        message = RGBDImage()
        message.header.stamp = color.header.stamp
        message.header.frame_id = color.header.frame_id
        message.rgb_camera_info = camera_info
        message.depth_camera_info = camera_info
        message.rgb = color
        message.depth = depth
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
        payload = compress_descriptor_matrix(result.features.descriptors)
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
        if abs(stamp - _stamp_seconds(depth)) > float(
            self.parameters["maximum_sensor_time_difference"]
        ):
            self._publish_status("DEGRADED", "SENSOR_TIME_MISMATCH")
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
            result = self.tracker.process(
                np.asarray(rgb),
                np.asarray(depth_image),
                _camera_intrinsics(camera_info),
                depth_scale,
                stamp,
                base_from_camera,
            )
            self._publish_diagnostics(result)
            self._publish_status(result.state, result.reason)
            if result.accepted:
                self._publish_odometry(result, color)
                self._publish_rgbd_features(result, color, depth, camera_info)
        except Exception as error:  # ROS boundary: report and keep the node diagnosable.
            self.get_logger().error(f"Visual frontend frame failed: {error}")
            self._publish_status("ERROR", type(error).__name__)
        finally:
            self.processing = False


def main(arguments: list[str] | None = None) -> None:
    """Run the learned visual odometry node until ROS shuts down."""
    rclpy.init(args=arguments)
    node = VisualOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
