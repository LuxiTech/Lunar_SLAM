#!/usr/bin/env python3
"""Replay a timed stereo-image segment directly from a ROS 2 SQLite bag.

This small diagnostic reader is intentionally independent of rosbag metadata
versions.  It reads, deserializes and publishes only the two requested image
topics; the source database is always opened read-only.
"""

import argparse
import math
from pathlib import Path
import sqlite3
import statistics
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rtabmap_msgs.msg import OdomInfo
from sensor_msgs.msg import Image


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="Path to the ROS 2 .db3 file")
    parser.add_argument("--start-offset", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--left-topic", default="/left_camera/image")
    parser.add_argument("--right-topic", default="/right_camera/image")
    return parser.parse_args()


class ReplayNode(Node):
    """Publish bag images while collecting odometry health metrics."""

    def __init__(self, left_topic: str, right_topic: str) -> None:
        super().__init__("luxi_sqlite_stereo_replay")
        self.left_publisher = self.create_publisher(
            Image, left_topic, qos_profile_sensor_data
        )
        self.right_publisher = self.create_publisher(
            Image, right_topic, qos_profile_sensor_data
        )
        self.poses: list[tuple[float, ...]] = []
        self.odom_info: list[tuple[bool, int, int, float, float]] = []
        self.reset_covariances = 0
        self.create_subscription(
            Odometry, "/rtabmap/odom", self._odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            OdomInfo, "/rtabmap/odom_info", self._info_callback, qos_profile_sensor_data
        )

    def _odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
        self.poses.append(
            (
                stamp,
                position.x,
                position.y,
                position.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        )
        self.reset_covariances += int(message.pose.covariance[0] >= 9999.0)

    def _info_callback(self, message: OdomInfo) -> None:
        self.odom_info.append(
            (
                message.lost,
                message.matches,
                message.inliers,
                message.inliers / max(1, message.matches),
                message.time_estimation * 1000.0,
            )
        )


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def quaternion_angle_degrees(first: tuple[float, ...], second: tuple[float, ...]) -> float:
    dot = abs(sum(first[index] * second[index] for index in range(4)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def report_odometry(node: ReplayNode) -> None:
    poses = node.poses
    infos = node.odom_info
    print(
        f"ODOM_METRICS samples={len(poses)} info={len(infos)} "
        f"reset_covariances={node.reset_covariances}"
    )
    if len(poses) > 1:
        translation_steps = [
            math.dist(first[1:4], second[1:4])
            for first, second in zip(poses, poses[1:])
        ]
        rotation_steps = [
            quaternion_angle_degrees(first[4:8], second[4:8])
            for first, second in zip(poses, poses[1:])
        ]
        duration = poses[-1][0] - poses[0][0]
        print(
            f"ODOM_MOTION duration={duration:.3f}s rate="
            f"{(len(poses) - 1) / duration:.2f}Hz path={sum(translation_steps):.3f}m "
            f"translation_p95={percentile(translation_steps, 0.95):.3f}m "
            f"translation_max={max(translation_steps):.3f}m "
            f"translation_jumps={sum(value > 0.5 for value in translation_steps)} "
            f"rotation_p95={percentile(rotation_steps, 0.95):.2f}deg "
            f"rotation_max={max(rotation_steps):.2f}deg "
            f"rotation_jumps={sum(value > 30.0 for value in rotation_steps)}"
        )
    if infos:
        matches = [float(value[1]) for value in infos]
        inliers = [float(value[2]) for value in infos]
        ratios = [value[3] for value in infos]
        estimation_times = [value[4] for value in infos]
        print(
            f"ODOM_TRACKING lost={sum(value[0] for value in infos)}/{len(infos)} "
            f"matches_median={statistics.median(matches):.1f} "
            f"matches_p05={percentile(matches, 0.05):.0f} "
            f"inliers_median={statistics.median(inliers):.1f} "
            f"inliers_p05={percentile(inliers, 0.05):.0f} "
            f"ratio_median={statistics.median(ratios):.3f} "
            f"ratio_p05={percentile(ratios, 0.05):.3f} "
            f"estimation_median={statistics.median(estimation_times):.2f}ms "
            f"estimation_p95={percentile(estimation_times, 0.95):.2f}ms"
        )


def main() -> int:
    arguments = parse_arguments()
    if not arguments.bag.is_file():
        raise FileNotFoundError(arguments.bag)
    if arguments.start_offset < 0.0 or arguments.duration <= 0.0 or arguments.rate <= 0.0:
        raise ValueError("start-offset must be non-negative; duration and rate must be positive")

    connection = sqlite3.connect(f"file:{arguments.bag}?mode=ro", uri=True)
    topic_ids = dict(connection.execute("SELECT name, id FROM topics"))
    selected = (arguments.left_topic, arguments.right_topic)
    missing = [topic for topic in selected if topic not in topic_ids]
    if missing:
        raise RuntimeError(f"Image topics not found in bag: {', '.join(missing)}")

    bag_start_ns = connection.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
    segment_start_ns = bag_start_ns + int(arguments.start_offset * 1.0e9)
    segment_end_ns = segment_start_ns + int(arguments.duration * 1.0e9)
    rows = connection.execute(
        """
        SELECT topic_id, timestamp, data
        FROM messages
        WHERE topic_id IN (?, ?) AND timestamp >= ? AND timestamp < ?
        ORDER BY timestamp
        """,
        (
            topic_ids[arguments.left_topic],
            topic_ids[arguments.right_topic],
            segment_start_ns,
            segment_end_ns,
        ),
    )

    rclpy.init()
    node = ReplayNode(arguments.left_topic, arguments.right_topic)
    publishers = {
        topic_ids[arguments.left_topic]: node.left_publisher,
        topic_ids[arguments.right_topic]: node.right_publisher,
    }
    # Give graph discovery time to connect before the first large image.
    discovery_deadline = time.monotonic() + 2.0
    while time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    first_record_ns = None
    playback_start = time.monotonic()
    published = 0
    for topic_id, record_ns, serialized in rows:
        if first_record_ns is None:
            first_record_ns = record_ns
        target = playback_start + (record_ns - first_record_ns) / 1.0e9 / arguments.rate
        delay = target - time.monotonic()
        if delay > 0.0:
            time.sleep(delay)
        publishers[topic_id].publish(deserialize_message(bytes(serialized), Image))
        # Drain both odometry topics without delaying the next camera frame.
        for _ in range(4):
            rclpy.spin_once(node, timeout_sec=0.0)
        published += 1

    drain_deadline = time.monotonic() + 2.0
    while time.monotonic() < drain_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.get_logger().info(
        f"Published {published} stereo image messages from offset "
        f"{arguments.start_offset:.3f}s for {arguments.duration:.3f}s"
    )
    report_odometry(node)
    node.destroy_node()
    rclpy.shutdown()
    connection.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
