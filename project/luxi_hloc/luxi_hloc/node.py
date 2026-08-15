"""ROS 2 node exposing HLoc coarse localization."""

from __future__ import annotations

import json
from pathlib import Path
import threading
from typing import Any

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import camera_matrix, invert_transform, quaternion_from_rotation
from .inference import HlocLocalizer, LocalizationOutput


def _stamp_seconds(message: Any) -> float:
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9


def _transform_matrix(message: Any) -> np.ndarray:
    translation = message.transform.translation
    rotation = message.transform.rotation
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("TF quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )
    matrix[:3, 3] = (translation.x, translation.y, translation.z)
    return matrix


class HlocLocalizerNode(Node):
    def __init__(self) -> None:
        super().__init__("luxi_hloc_localizer")
        defaults = {
            "map_directory": "",
            "device": "auto",
            "resize_max": 640,
            "max_keypoints": 1024,
            "cpu_threads": 4,
            "top_k": 5,
            "color_topic": "/sensors/rgbd/color/image_raw",
            "depth_topic": "/sensors/rgbd/depth/image_raw",
            "camera_info_topic": "/sensors/rgbd/color/camera_info",
            "coarse_pose_topic": "/luxi_hloc/coarse_pose",
            "status_topic": "/luxi_hloc/status",
            "diagnostics_topic": "/luxi_hloc/diagnostics",
            "map_frame": "map",
            "base_frame": "base_link",
            "query_period": 1.0,
            "maximum_sensor_time_difference": 0.05,
            "depth_scale_16u": 0.001,
            "minimum_retrieval_score": 0.05,
            "minimum_matches": 25,
            "minimum_landmarks": 12,
            "minimum_inliers": 8,
            "minimum_inlier_ratio": 0.25,
            "maximum_reprojection_rmse": 3.0,
            "ransac_reprojection_error": 4.0,
            "ransac_iterations": 1000,
            "ransac_confidence": 0.999,
            "require_depth_verification": True,
            "minimum_depth_verified": 6,
            "maximum_median_depth_residual": 0.25,
            "minimum_depth": 0.25,
            "maximum_depth": 6.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        parameters = {name: self.get_parameter(name).value for name in defaults}
        if not parameters["map_directory"]:
            raise RuntimeError("The map_directory parameter is required")
        map_directory = Path(parameters["map_directory"])
        if not map_directory.is_dir():
            raise RuntimeError(f"HLoc map directory does not exist: {map_directory}")
        if parameters["query_period"] <= 0.0:
            raise ValueError("query_period must be positive")
        if parameters["maximum_sensor_time_difference"] < 0.0:
            raise ValueError("maximum_sensor_time_difference cannot be negative")

        self.status_publisher = self.create_publisher(
            String,
            parameters["status_topic"],
            10,
        )
        self.diagnostics_publisher = self.create_publisher(
            String,
            parameters["diagnostics_topic"],
            10,
        )
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            parameters["coarse_pose_topic"],
            10,
        )
        self.get_logger().info(f"Loading HLoc map and models from {map_directory}")
        self.localizer = HlocLocalizer(map_directory, parameters)
        self.get_logger().info(f"HLoc inference device: {self.localizer.device}")

        self.parameters = parameters
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.latest_color: Image | None = None
        self.latest_depth: Image | None = None
        self.latest_camera_info: CameraInfo | None = None
        self.enabled = True
        self.processing = False
        self.force_query = False

        self.create_subscription(
            Image,
            parameters["color_topic"],
            self._color_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            parameters["depth_topic"],
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            parameters["camera_info_topic"],
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_service(SetBool, "~/enable", self._enable_callback)
        self.create_service(Trigger, "~/relocalize", self._relocalize_callback)
        self.timer = self.create_timer(parameters["query_period"], self._process_latest)
        self._publish_status("WAITING_FOR_SENSOR_DATA")

    def _publish_status(self, status: str) -> None:
        message = String()
        message.data = status
        self.status_publisher.publish(message)

    def _publish_diagnostics(self, output: LocalizationOutput) -> None:
        data: dict[str, Any] = {
            "accepted": output.accepted,
            "reason": output.reason,
            "reference": output.reference_name,
            "retrieval_score": output.retrieval_score,
            "elapsed_seconds": output.elapsed_seconds,
            "candidates_tested": output.candidates_tested,
            "device": self.localizer.device,
        }
        if output.candidate is not None:
            data.update(
                {
                    "matches": output.candidate.match_count,
                    "landmarks": output.candidate.valid_landmark_count,
                    "inliers": output.candidate.inlier_count,
                    "inlier_ratio": output.candidate.inlier_ratio,
                    "depth_verified": output.candidate.depth_verified_count,
                    "median_depth_residual": output.candidate.median_depth_residual,
                    "reprojection_rmse": (
                        output.candidate.pnp.reprojection_rmse
                        if output.candidate.pnp is not None
                        else None
                    ),
                }
            )
        message = String()
        message.data = json.dumps(data, ensure_ascii=False)
        self.diagnostics_publisher.publish(message)

    def _color_callback(self, message: Image) -> None:
        with self.lock:
            self.latest_color = message

    def _depth_callback(self, message: Image) -> None:
        with self.lock:
            self.latest_depth = message

    def _camera_info_callback(self, message: CameraInfo) -> None:
        with self.lock:
            self.latest_camera_info = message

    def _enable_callback(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self.enabled = request.data
        response.success = True
        response.message = "enabled" if self.enabled else "disabled"
        self._publish_status("WAITING_FOR_SENSOR_DATA" if self.enabled else "DISABLED")
        return response

    def _relocalize_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        self.force_query = True
        response.success = True
        response.message = "localization requested"
        return response

    def _snapshot(self) -> tuple[Image, Image, CameraInfo] | None:
        with self.lock:
            if (
                self.latest_color is None
                or self.latest_depth is None
                or self.latest_camera_info is None
            ):
                return None
            return self.latest_color, self.latest_depth, self.latest_camera_info

    def _process_latest(self) -> None:
        if not self.enabled or self.processing:
            return
        snapshot = self._snapshot()
        if snapshot is None:
            self._publish_status("WAITING_FOR_SENSOR_DATA")
            return
        color_message, depth_message, camera_info = snapshot
        maximum_difference = self.parameters["maximum_sensor_time_difference"]
        if abs(_stamp_seconds(color_message) - _stamp_seconds(depth_message)) > maximum_difference:
            self._publish_status("SENSOR_TIME_MISMATCH")
            return
        if color_message.width != camera_info.width or color_message.height != camera_info.height:
            self._publish_status("CAMERA_INFO_SIZE_MISMATCH")
            return

        self.processing = True
        self.force_query = False
        self._publish_status("PROCESSING")
        try:
            rgb = self.bridge.imgmsg_to_cv2(color_message, desired_encoding="rgb8")
            depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
            if depth.ndim != 2:
                raise ValueError("depth image must be single-channel")
            if depth.dtype == np.uint16:
                depth_scale = self.parameters["depth_scale_16u"]
            elif depth.dtype == np.float32:
                depth_scale = 1.0
            else:
                raise ValueError(f"unsupported depth dtype: {depth.dtype}")
            intrinsics = camera_matrix(
                camera_info.k[0],
                camera_info.k[4],
                camera_info.k[2],
                camera_info.k[5],
            )
            output = self.localizer.localize(rgb, depth, intrinsics, depth_scale)
            self._publish_diagnostics(output)
            if not output.accepted or output.map_from_camera is None:
                self._publish_status(output.reason)
                return
            try:
                base_from_camera_message = self.tf_buffer.lookup_transform(
                    self.parameters["base_frame"],
                    color_message.header.frame_id,
                    Time.from_msg(color_message.header.stamp),
                    timeout=Duration(seconds=0.2),
                )
            except TransformException as exception:
                self.get_logger().warning(f"Camera TF unavailable: {exception}")
                self._publish_status("CAMERA_TF_UNAVAILABLE")
                return
            base_from_camera = _transform_matrix(base_from_camera_message)
            map_from_base = output.map_from_camera @ invert_transform(base_from_camera)
            self._publish_pose(map_from_base, color_message)
            self._publish_status("LOCALIZED")
        except Exception as exception:  # Keep diagnostics alive for recoverable sensor/model errors.
            self.get_logger().error(f"HLoc processing failed: {exception}")
            self._publish_status(f"ERROR: {exception}")
        finally:
            self.processing = False

    def _publish_pose(self, map_from_base: np.ndarray, source: Image) -> None:
        message = PoseWithCovarianceStamped()
        message.header.stamp = source.header.stamp
        message.header.frame_id = self.parameters["map_frame"]
        message.pose.pose.position.x = float(map_from_base[0, 3])
        message.pose.pose.position.y = float(map_from_base[1, 3])
        message.pose.pose.position.z = float(map_from_base[2, 3])
        quaternion = quaternion_from_rotation(map_from_base[:3, :3])
        message.pose.pose.orientation.x = float(quaternion[0])
        message.pose.pose.orientation.y = float(quaternion[1])
        message.pose.pose.orientation.z = float(quaternion[2])
        message.pose.pose.orientation.w = float(quaternion[3])
        covariance = [0.0] * 36
        covariance[0] = 0.10
        covariance[7] = 0.10
        covariance[14] = 0.20
        covariance[21] = 0.05
        covariance[28] = 0.05
        covariance[35] = 0.10
        message.pose.covariance = covariance
        self.pose_publisher.publish(message)


def main(arguments: list[str] | None = None) -> None:
    rclpy.init(args=arguments)
    node = None
    try:
        # Model loading is intentionally inside the interrupt guard: the
        # first localization start can spend several seconds parsing NetVLAD,
        # and stopping from the web page during that window is normal.
        node = HlocLocalizerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()
