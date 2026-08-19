#!/usr/bin/env python3
"""Print SGBM depth at the 3D point selected in RViz Publish Point."""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

# Importing this package registers PointStamped conversions with tf2 in ROS 2.
import tf2_geometry_msgs  # noqa: F401


class RvizDepthClickProbe(Node):
    """Project a clicked RViz point to the rectified left depth image."""

    def __init__(self):
        super().__init__('rviz_depth_click_probe')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('depth_topic', '/stereo/depth')
        self.declare_parameter('camera_info_topic', '/stereo/left/camera_info')
        self.declare_parameter('search_radius_px', 2)
        self.declare_parameter('min_depth_m', 0.45)
        self.declare_parameter('max_depth_m', 4.5)

        self.search_radius = max(0, int(self.get_parameter('search_radius_px').value))
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.depth = None
        self.depth_header = None
        self.camera_info = None

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value), self.depth_callback, sensor_qos)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self.camera_info_callback, QoSProfile(depth=5))
        self.create_subscription(
            PointStamped, str(self.get_parameter('clicked_point_topic').value),
            self.clicked_point_callback, QoSProfile(depth=5))
        self.get_logger().info(
            'Ready. In RViz select the Publish Point tool and click a point on '
            '/stereo/points. Measurements are printed here.')

    def depth_callback(self, msg):
        if msg.encoding != '32FC1':
            self.get_logger().error('Expected 32FC1 on depth topic, received %s.' % msg.encoding)
            return
        expected_bytes = msg.width * msg.height * 4
        if len(msg.data) < expected_bytes:
            self.get_logger().warning('Ignoring truncated depth image.')
            return
        self.depth = np.frombuffer(
            msg.data, dtype=np.float32, count=msg.width * msg.height).reshape(msg.height, msg.width).copy()
        self.depth_header = msg.header

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def clicked_point_callback(self, msg):
        if self.depth is None or self.depth_header is None or self.camera_info is None:
            self.get_logger().warning('Waiting for /stereo/depth and /stereo/left/camera_info.')
            return

        camera_frame = self.depth_header.frame_id or self.camera_info.header.frame_id
        if not camera_frame:
            self.get_logger().error('Depth and CameraInfo have no frame_id.')
            return
        try:
            clicked_camera = self.tf_buffer.transform(
                msg, camera_frame, timeout=Duration(seconds=0.5))
        except TransformException as exc:
            self.get_logger().error(
                'Cannot transform clicked_point from %s to %s: %s'
                % (msg.header.frame_id, camera_frame, exc))
            return

        p = clicked_camera.point
        if p.z <= 1e-6:
            self.get_logger().warning(
                'Clicked point is behind the camera (z=%.3f). Click a visible point cloud sample.' % p.z)
            return
        # P describes this already-rectified left image. Use it instead of K.
        fx, fy, cx, cy = (
            self.camera_info.p[0], self.camera_info.p[5],
            self.camera_info.p[2], self.camera_info.p[6])
        if fx <= 0.0 or fy <= 0.0:
            self.get_logger().error('CameraInfo has invalid rectified focal lengths.')
            return
        u = int(round(fx * p.x / p.z + cx))
        v = int(round(fy * p.y / p.z + cy))
        if not (0 <= u < self.depth.shape[1] and 0 <= v < self.depth.shape[0]):
            self.get_logger().warning(
                'Projected pixel (%d, %d) is outside %dx%d depth image.'
                % (u, v, self.depth.shape[1], self.depth.shape[0]))
            return

        radius = self.search_radius
        patch = self.depth[
            max(0, v - radius):min(self.depth.shape[0], v + radius + 1),
            max(0, u - radius):min(self.depth.shape[1], u + radius + 1),
        ]
        valid = patch[
            np.isfinite(patch)
            & (patch >= self.min_depth_m)
            & (patch <= self.max_depth_m)
        ]
        if valid.size == 0:
            self.get_logger().warning(
                'No valid SGBM depth in %dx%d pixels around (%d, %d). '
                'Click a textured target or enlarge search_radius_px.'
                % (patch.shape[1], patch.shape[0], u, v))
            return

        depth_z = float(np.median(valid))
        x = (u - cx) * depth_z / fx
        y = (v - cy) * depth_z / fy
        range_m = math.sqrt(x * x + y * y + depth_z * depth_z)
        self.get_logger().info(
            'SGBM MEASURE: pixel=(%d,%d) patch=%dx%d valid=%d/%d '
            'Z_depth=%.3f m range=%.3f m '
            'RViz_clicked=(%.3f,%.3f,%.3f) %s'
            % (u, v, patch.shape[1], patch.shape[0], valid.size, patch.size,
               depth_z, range_m, p.x, p.y, p.z, camera_frame))


def main():
    rclpy.init()
    node = RvizDepthClickProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
