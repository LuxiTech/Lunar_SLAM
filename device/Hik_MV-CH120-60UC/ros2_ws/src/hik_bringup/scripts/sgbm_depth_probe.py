#!/usr/bin/env python3
"""Measure the live stereo-depth stream without changing the mapping pipeline.

Example:
  ros2 run hik_bringup sgbm_depth_probe.py --ros-args -p duration_sec:=15.0
"""

import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class SgbmDepthProbe(Node):
    """Collect a bounded, reproducible summary of /stereo/depth."""

    def __init__(self):
        super().__init__('sgbm_depth_probe')
        self.declare_parameter('depth_topic', '/stereo/depth')
        self.declare_parameter('duration_sec', 15.0)
        self.declare_parameter('min_depth_m', 0.01)
        self.declare_parameter('max_depth_m', 100.0)
        self.declare_parameter('report_every_frames', 0)
        # Set all four ROI values to measure only the target panel. A negative
        # x/y or a non-positive width/height means the complete depth image.
        self.declare_parameter('roi_x', -1)
        self.declare_parameter('roi_y', -1)
        self.declare_parameter('roi_width', 0)
        self.declare_parameter('roi_height', 0)

        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.duration_sec = max(1.0, float(self.get_parameter('duration_sec').value))
        self.min_depth_m = max(0.0, float(self.get_parameter('min_depth_m').value))
        self.max_depth_m = max(self.min_depth_m, float(self.get_parameter('max_depth_m').value))
        self.report_every = max(0, int(self.get_parameter('report_every_frames').value))
        self.roi_x = int(self.get_parameter('roi_x').value)
        self.roi_y = int(self.get_parameter('roi_y').value)
        self.roi_width = int(self.get_parameter('roi_width').value)
        self.roi_height = int(self.get_parameter('roi_height').value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.samples = []
        self.intervals_sec = []
        self.last_stamp_sec = None
        self.bad_encoding_count = 0
        self.create_subscription(Image, self.depth_topic, self.image_callback, sensor_qos)
        self.get_logger().info(
            'Measuring %s for %.1f s (accepted range %.2f to %.2f m, ROI=%s).'
            % (self.depth_topic, self.duration_sec, self.min_depth_m, self.max_depth_m,
               'full image' if self.roi_width <= 0 or self.roi_height <= 0
               else '%d,%d %dx%d' % (self.roi_x, self.roi_y, self.roi_width, self.roi_height)))

    def image_callback(self, msg):
        if msg.encoding != '32FC1':
            self.bad_encoding_count += 1
            if self.bad_encoding_count == 1:
                self.get_logger().error(
                    'Expected 32FC1 depth, received %s. Check depth_topic.' % msg.encoding)
            return
        expected_bytes = msg.height * msg.width * 4
        if len(msg.data) < expected_bytes:
            self.get_logger().warning(
                'Ignoring truncated image: received %d, expected at least %d bytes.'
                % (len(msg.data), expected_bytes))
            return

        depth = np.frombuffer(
            msg.data, dtype=np.float32, count=msg.height * msg.width).reshape(msg.height, msg.width)
        x0 = max(0, self.roi_x)
        y0 = max(0, self.roi_y)
        if self.roi_x < 0 or self.roi_y < 0 or self.roi_width <= 0 or self.roi_height <= 0:
            x0, y0, x1, y1 = 0, 0, msg.width, msg.height
        else:
            x1 = min(msg.width, x0 + self.roi_width)
            y1 = min(msg.height, y0 + self.roi_height)
        roi = depth[y0:y1, x0:x1]
        valid = roi[
            np.isfinite(roi)
            & (roi >= self.min_depth_m)
            & (roi <= self.max_depth_m)
        ]
        total_pixels = max(1, roi.size)
        self.samples.append({
            'width': msg.width,
            'height': msg.height,
            'roi_x': x0,
            'roi_y': y0,
            'roi_width': max(0, x1 - x0),
            'roi_height': max(0, y1 - y0),
            'valid_ratio': valid.size / total_pixels,
            'median_m': float(np.median(valid)) if valid.size else float('nan'),
            'p10_m': float(np.percentile(valid, 10)) if valid.size else float('nan'),
            'p90_m': float(np.percentile(valid, 90)) if valid.size else float('nan'),
        })

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp_sec is not None and stamp_sec > self.last_stamp_sec:
            self.intervals_sec.append(stamp_sec - self.last_stamp_sec)
        self.last_stamp_sec = stamp_sec

        if self.report_every and len(self.samples) % self.report_every == 0:
            self.get_logger().info(
                'frame=%d valid=%.2f%% median=%.3f m'
                % (len(self.samples), self.samples[-1]['valid_ratio'] * 100.0,
                   self.samples[-1]['median_m']))

    def report(self):
        if not self.samples:
            self.get_logger().error(
                'No frames received from %s. Start camera_only.launch.py or '
                'rtabmap_stereo_imu.launch.py first.' % self.depth_topic)
            return False

        ratios = np.array([item['valid_ratio'] for item in self.samples])
        medians = np.array([item['median_m'] for item in self.samples])
        p10s = np.array([item['p10_m'] for item in self.samples])
        p90s = np.array([item['p90_m'] for item in self.samples])
        last = self.samples[-1]
        self.get_logger().info(
            'DEPTH RESULT: frames=%d resolution=%dx%d roi=%d,%d %dx%d valid_ratio '
            'median=%.2f%% mean=%.2f%% range=[%.2f%%, %.2f%%]'
            % (len(self.samples), last['width'], last['height'], last['roi_x'], last['roi_y'],
               last['roi_width'], last['roi_height'],
               np.nanmedian(ratios) * 100.0, np.nanmean(ratios) * 100.0,
               np.nanmin(ratios) * 100.0, np.nanmax(ratios) * 100.0))
        self.get_logger().info(
            'DEPTH RESULT: depth_m median=%.3f p10=%.3f p90=%.3f'
            % (np.nanmedian(medians), np.nanmedian(p10s), np.nanmedian(p90s)))
        if self.intervals_sec:
            intervals = np.array(self.intervals_sec)
            self.get_logger().info(
                'DEPTH RESULT: publish_hz=%.2f interval_ms median=%.1f p90=%.1f'
                % (1.0 / np.mean(intervals), np.median(intervals) * 1000.0,
                   np.percentile(intervals, 90) * 1000.0))
        else:
            self.get_logger().warning('Only one valid timestamp received; cannot compute publish rate.')
        return True


def main():
    rclpy.init()
    node = SgbmDepthProbe()
    deadline = time.monotonic() + node.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.report()
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
