#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy._rclpy_pybind11 import RCLError
from tf2_ros import TransformBroadcaster


class OdomTfRepublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_republisher')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('max_valid_covariance', 1000.0)
        self.declare_parameter('start_with_identity', True)

        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.max_valid_covariance = float(self.get_parameter('max_valid_covariance').value)
        publish_rate_hz = max(1.0, float(self.get_parameter('publish_rate_hz').value))

        self.broadcaster = TransformBroadcaster(self)
        self.latest = None

        if bool(self.get_parameter('start_with_identity').value):
            self.latest = self.make_identity_transform()

        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            qos)
        self.create_timer(1.0 / publish_rate_hz, self.publish_latest)
        self.get_logger().info(
            'Republishing odom TF: %s -> %s at %.1f Hz'
            % (self.odom_frame_id, self.base_frame_id, publish_rate_hz))

    def make_identity_transform(self):
        msg = TransformStamped()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_frame_id
        msg.transform.rotation.w = 1.0
        return msg

    def odom_callback(self, msg):
        if not self.is_valid_odom(msg):
            return

        transform = TransformStamped()
        transform.header.frame_id = msg.header.frame_id or self.odom_frame_id
        transform.child_frame_id = msg.child_frame_id or self.base_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.latest = transform

    def is_valid_odom(self, msg):
        q = msg.pose.pose.orientation
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm < 0.5:
            return False

        covariance = msg.pose.covariance
        diagonal = [covariance[i] for i in (0, 7, 14, 21, 28, 35)]
        return all(value < self.max_valid_covariance for value in diagonal)

    def publish_latest(self):
        if self.latest is None:
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.latest.header.frame_id
        transform.child_frame_id = self.latest.child_frame_id
        transform.transform = self.latest.transform
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = OdomTfRepublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
