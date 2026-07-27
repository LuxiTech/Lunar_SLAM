#!/usr/bin/env python3

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy._rclpy_pybind11 import RCLError
from rtabmap_msgs.msg import Info, MapData
from sensor_msgs.msg import PointCloud2


class RtabmapStatus(Node):
    def __init__(self):
        super().__init__('rtabmap_status')
        self.declare_parameter('info_topic', '/info')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_data_topic', '/mapData')
        self.declare_parameter('cloud_map_topic', '/cloud_map')
        self.declare_parameter('print_rate_hz', 1.0)

        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        reliable_qos = QoSProfile(depth=5)

        self.last_info = None
        self.last_odom = None
        self.first_odom = None
        self.last_map_data = None
        self.last_cloud_width = None

        self.create_subscription(
            Info, self.get_parameter('info_topic').value, self.info_cb, qos)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self.odom_cb, qos)
        self.create_subscription(
            MapData, self.get_parameter('map_data_topic').value, self.map_data_cb, reliable_qos)
        self.create_subscription(
            PointCloud2, self.get_parameter('cloud_map_topic').value, self.cloud_map_cb, reliable_qos)

        rate = max(0.2, float(self.get_parameter('print_rate_hz').value))
        self.create_timer(1.0 / rate, self.print_status)

    def info_cb(self, msg):
        self.last_info = msg

    def odom_cb(self, msg):
        self.last_odom = msg
        if self.first_odom is None:
            self.first_odom = msg

    def map_data_cb(self, msg):
        self.last_map_data = msg

    def cloud_map_cb(self, msg):
        self.last_cloud_width = msg.width * max(1, msg.height)

    @staticmethod
    def distance(a, b):
        pa = a.pose.pose.position
        pb = b.pose.pose.position
        return math.sqrt(
            (pa.x - pb.x) ** 2 +
            (pa.y - pb.y) ** 2 +
            (pa.z - pb.z) ** 2)

    def print_status(self):
        if self.last_info is None:
            self.get_logger().warning(
                'No /info yet: RTAB-Map has not published mapping status. '
                'Check /stereo/rgbd_image and /odom first.')
            return

        odom_delta = 0.0
        if self.first_odom is not None and self.last_odom is not None:
            odom_delta = self.distance(self.first_odom, self.last_odom)

        map_nodes = 0
        map_clouds = 0
        if self.last_map_data is not None:
            map_nodes = len(self.last_map_data.graph.poses_id)
            map_clouds = len(self.last_map_data.nodes)

        self.get_logger().info(
            'RTAB-Map status: ref_id=%d wm_nodes=%d odom_delta=%.3f m '
            'mapData_poses=%d mapData_clouds=%d cloud_map_points=%s'
            % (
                self.last_info.ref_id,
                len(self.last_info.wm_state),
                odom_delta,
                map_nodes,
                map_clouds,
                'none' if self.last_cloud_width is None else str(self.last_cloud_width),
            )
        )


def main():
    rclpy.init()
    node = RtabmapStatus()
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
