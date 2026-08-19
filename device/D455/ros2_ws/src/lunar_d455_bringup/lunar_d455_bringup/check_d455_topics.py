#!/usr/bin/env python3
"""Verify the D455 streams required by the project adapter."""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_msgs.msg import TFMessage


SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
TF_STATIC_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


@dataclass(frozen=True)
class TopicSpec:
    name: str
    msg_type: type
    qos: QoSProfile


REQUIRED_TOPICS = (
    TopicSpec("/camera/camera/color/image_raw", Image, SENSOR_QOS),
    TopicSpec("/camera/camera/aligned_depth_to_color/image_raw", Image, SENSOR_QOS),
    TopicSpec("/camera/camera/color/camera_info", CameraInfo, SENSOR_QOS),
    TopicSpec("/camera/camera/imu", Imu, SENSOR_QOS),
    TopicSpec("/tf_static", TFMessage, TF_STATIC_QOS),
)


class D455TopicChecker(Node):
    """Collect one valid sample from each D455 adapter input."""

    def __init__(self) -> None:
        super().__init__("d455_topic_checker")
        self._seen: Dict[str, int] = {spec.name: 0 for spec in REQUIRED_TOPICS}
        self._images: Dict[str, tuple[int, int]] = {}
        for spec in REQUIRED_TOPICS:
            self.create_subscription(
                spec.msg_type,
                spec.name,
                lambda msg, topic=spec.name: self._receive(topic, msg),
                spec.qos,
            )

    def _receive(self, topic: str, message) -> None:
        if hasattr(message, "header"):
            stamp = message.header.stamp
            if not message.header.frame_id or (stamp.sec == 0 and stamp.nanosec == 0):
                return
        if isinstance(message, Image):
            self._images[topic] = (message.width, message.height)
        self._seen[topic] += 1

    @property
    def missing_topics(self) -> list[str]:
        return [topic for topic, count in self._seen.items() if count == 0]

    @property
    def aligned_dimensions_match(self) -> bool:
        color = self._images.get("/camera/camera/color/image_raw")
        depth = self._images.get("/camera/camera/aligned_depth_to_color/image_raw")
        return color is not None and color == depth


def main() -> int:
    rclpy.init()
    node = D455TopicChecker()
    deadline = time.monotonic() + 20.0
    try:
        while time.monotonic() < deadline and (
            node.missing_topics or not node.aligned_dimensions_match
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.missing_topics:
            node.get_logger().error(
                "Missing required topics: " + ", ".join(node.missing_topics)
            )
            return 1
        if not node.aligned_dimensions_match:
            node.get_logger().error("Aligned depth dimensions do not match color")
            return 1
        node.get_logger().info("D455 aligned RGB-D, CameraInfo, IMU and TF passed.")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
