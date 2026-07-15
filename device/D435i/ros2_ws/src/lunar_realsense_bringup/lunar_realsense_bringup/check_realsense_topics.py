#!/usr/bin/env python3
"""Smoke-test RealSense RGB, depth, point cloud, and TF publications."""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from tf2_msgs.msg import TFMessage


@dataclass(frozen=True)
class TopicSpec:
    name: str
    msg_type: type
    qos: QoSProfile


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


REQUIRED_TOPICS = (
    TopicSpec("/camera/camera/color/image_raw", Image, SENSOR_QOS),
    TopicSpec("/camera/camera/depth/image_rect_raw", Image, SENSOR_QOS),
    TopicSpec("/camera/camera/depth/color/points", PointCloud2, SENSOR_QOS),
    TopicSpec("/tf_static", TFMessage, TF_STATIC_QOS),
)


class RealSenseTopicChecker(Node):
    def __init__(self) -> None:
        super().__init__("realsense_topic_checker")
        self._seen: Dict[str, int] = {spec.name: 0 for spec in REQUIRED_TOPICS}
        for spec in REQUIRED_TOPICS:
            self.create_subscription(
                spec.msg_type,
                spec.name,
                lambda _msg, topic=spec.name: self._mark_seen(topic),
                spec.qos,
            )

    def _mark_seen(self, topic: str) -> None:
        self._seen[topic] += 1

    @property
    def missing_topics(self) -> list[str]:
        return [topic for topic, count in self._seen.items() if count == 0]

    @property
    def seen_counts(self) -> Dict[str, int]:
        return dict(self._seen)


def main() -> int:
    rclpy.init()
    node = RealSenseTopicChecker()
    deadline = time.monotonic() + 20.0

    try:
        while time.monotonic() < deadline and node.missing_topics:
            rclpy.spin_once(node, timeout_sec=0.2)

        for topic, count in node.seen_counts.items():
            node.get_logger().info(f"{topic}: {count} message(s)")

        missing = node.missing_topics
        if missing:
            node.get_logger().error(f"Missing required topics: {', '.join(missing)}")
            return 1

        node.get_logger().info("RealSense RGB, depth, point cloud, and TF smoke test passed.")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
