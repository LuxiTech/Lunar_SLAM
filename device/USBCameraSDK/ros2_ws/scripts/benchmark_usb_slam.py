#!/usr/bin/env python3
"""Measure static USB SLAM drift and Jetson resource use on a live graph."""

from __future__ import annotations

import argparse
from collections import defaultdict
import math
import os
from pathlib import Path
import re
import signal
import statistics
import subprocess
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data


MODULES = {
    "camera": "/stereo_node",
    "depth": "/stereo_depth_node",
    "frontend": "/visual_odometry_node",
    "odometry": "/rgbd_odometry",
    "rtabmap": "/rtabmap",
    "adapter": "/sensor_adapter_node",
}


def percentile(values: list[float], level: float) -> float:
    """Return a percentile, or NaN when no values were collected."""
    return float(np.percentile(values, level)) if values else float("nan")


def process_module(pid: int) -> str | None:
    """Classify a ROS process using its executable path, excluding wrappers."""
    try:
        executable = os.readlink(f"/proc/{pid}/exe")
    except OSError:
        return None
    if Path(executable).name.startswith("python3"):
        try:
            command = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ")
        except OSError:
            command = b""
        if b"/luxi_visual_frontend/visual_odometry_node" in command:
            return "frontend"
        if b"/usb_camera_driver/crestereo_depth_node" in command:
            return "depth"
    for name, suffix in MODULES.items():
        if executable.endswith(suffix):
            return name
    return None


class ResourceSampler:
    """Collect tegrastats plus per-process CPU and RSS samples."""

    def __init__(self) -> None:
        self.stop_event = threading.Event()
        self.cpu: dict[str, list[float]] = defaultdict(list)
        self.rss: dict[str, list[float]] = defaultdict(list)
        self.gpu: list[float] = []
        self.power: list[float] = []
        self.ram: list[float] = []
        self._thread = threading.Thread(target=self._sample_processes, daemon=True)
        self._tegrastats: subprocess.Popen[str] | None = None
        self._tegrastats_thread: threading.Thread | None = None

    def start(self) -> None:
        self._tegrastats = subprocess.Popen(
            ["tegrastats", "--interval", "250"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            start_new_session=True,
        )
        self._tegrastats_thread = threading.Thread(
            target=self._read_tegrastats, daemon=True
        )
        self._tegrastats_thread.start()
        self._thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        self._thread.join(timeout=2.0)
        if self._tegrastats is not None and self._tegrastats.poll() is None:
            os.killpg(self._tegrastats.pid, signal.SIGINT)
            try:
                self._tegrastats.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                os.killpg(self._tegrastats.pid, signal.SIGTERM)
                self._tegrastats.wait(timeout=2.0)
        if self._tegrastats_thread is not None:
            self._tegrastats_thread.join(timeout=1.0)

    def _read_tegrastats(self) -> None:
        assert self._tegrastats is not None and self._tegrastats.stdout is not None
        for line in self._tegrastats.stdout:
            gpu = re.search(r"GR3D_FREQ (\d+)%", line)
            power = re.search(r"VDD_IN (\d+)mW", line)
            ram = re.search(r"RAM (\d+)/", line)
            if gpu:
                self.gpu.append(float(gpu.group(1)))
            if power:
                self.power.append(float(power.group(1)) / 1000.0)
            if ram:
                self.ram.append(float(ram.group(1)) / 1024.0)

    @staticmethod
    def _process_snapshot() -> dict[tuple[str, int], tuple[int, int]]:
        snapshot = {}
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            module = process_module(pid)
            if module is None:
                continue
            try:
                fields = (entry / "stat").read_text().split()
                resident_pages = int(fields[23])
                ticks = int(fields[13]) + int(fields[14])
            except (OSError, ValueError, IndexError):
                continue
            snapshot[(module, pid)] = (ticks, resident_pages)
        return snapshot

    def _sample_processes(self) -> None:
        ticks_per_second = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
        page_size = os.sysconf("SC_PAGE_SIZE")
        previous = self._process_snapshot()
        previous_time = time.monotonic()
        while not self.stop_event.wait(0.25):
            current_time = time.monotonic()
            current = self._process_snapshot()
            elapsed = current_time - previous_time
            by_module_cpu: dict[str, float] = defaultdict(float)
            by_module_rss: dict[str, float] = defaultdict(float)
            for key, (ticks, pages) in current.items():
                module, _ = key
                by_module_rss[module] += pages * page_size / 1024.0**3
                if key in previous and elapsed > 0.0:
                    by_module_cpu[module] += (
                        ticks - previous[key][0]
                    ) / ticks_per_second / elapsed
            for module, value in by_module_cpu.items():
                self.cpu[module].append(value)
            for module, value in by_module_rss.items():
                self.rss[module].append(value)
            previous = current
            previous_time = current_time


def quaternion_angle(first: np.ndarray, second: np.ndarray) -> float:
    """Return the shortest angular distance between unit quaternions."""
    dot = abs(float(np.dot(first, second)))
    return math.degrees(2.0 * math.acos(np.clip(dot, -1.0, 1.0)))


def main() -> None:
    """Collect 15 seconds after a short warm-up and print comparable metrics."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--odom-topic",
        default="/rtabmap/odom",
        help="Odometry topic produced by the profile under test.",
    )
    parser.add_argument("--warmup", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=15.0)
    arguments = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("usb_slam_benchmark")
    odometry: list[tuple[float, np.ndarray, np.ndarray]] = []
    diagnostics: list[dict[str, str]] = []

    def receive_odom(message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        odometry.append((
            time.monotonic(),
            np.array([position.x, position.y, position.z]),
            np.array([orientation.x, orientation.y, orientation.z, orientation.w]),
        ))

    def receive_diagnostics(message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name == "luxi_visual_frontend/tracking":
                values = {entry.key: entry.value for entry in status.values}
                values["accepted"] = status.message
                diagnostics.append(values)

    node.create_subscription(
        Odometry, arguments.odom_topic, receive_odom,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        DiagnosticArray, "/luxi_visual_frontend/diagnostics",
        receive_diagnostics, qos_profile_sensor_data,
    )

    warmup_deadline = time.monotonic() + arguments.warmup
    while time.monotonic() < warmup_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    odometry.clear()
    diagnostics.clear()
    resources = ResourceSampler()
    resources.start()
    deadline = time.monotonic() + arguments.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    resources.stop()

    if len(odometry) < 2:
        raise RuntimeError(f"received only {len(odometry)} odometry messages")
    times = [sample[0] for sample in odometry]
    positions = np.stack([sample[1] for sample in odometry])
    orientations = np.stack([sample[2] for sample in odometry])
    translations = np.linalg.norm(positions - positions[0], axis=1) * 1000.0
    rotations = [quaternion_angle(orientations[0], value) for value in orientations]
    intervals = np.diff(times)
    # The learned frontend reports fused successes as ACCEPTED_IMU_DEPTH.
    # Treat every ACCEPTED* state as a successful tracking update.
    accepted = [
        item for item in diagnostics
        if item.get("accepted", "").startswith("ACCEPTED")
    ]
    numeric = lambda key: [float(item[key]) for item in accepted if key in item]

    print(f"odom_samples={len(odometry)} diagnostics={len(diagnostics)}")
    print(
        f"odom_hz_mean={1.0 / statistics.mean(intervals):.3f} "
        f"median={1.0 / statistics.median(intervals):.3f}"
    )
    print(
        f"drift_end_mm={translations[-1]:.3f} max_mm={max(translations):.3f} "
        f"rotation_end_deg={rotations[-1]:.4f} max_deg={max(rotations):.4f}"
    )
    if diagnostics:
        print(
            f"tracking_accepted={len(accepted)}/{len(diagnostics)} "
            f"inliers_mean={statistics.mean(numeric('inliers')):.1f} "
            f"ratio_mean={statistics.mean(numeric('inlier_ratio')):.5f} "
            f"rmse_mean_px={statistics.mean(numeric('reprojection_rmse')):.3f} "
            f"elapsed_mean_ms={statistics.mean(numeric('elapsed_seconds')) * 1000.0:.1f} "
            f"extract_mean_ms={statistics.mean(numeric('extract_seconds')) * 1000.0:.1f} "
            f"match_mean_ms={statistics.mean(numeric('match_seconds')) * 1000.0:.1f} "
            f"match_layers_mean={statistics.mean(numeric('match_layers')):.1f} "
            f"keypoints_mean={statistics.mean(numeric('keypoints')):.1f} "
            f"matches_mean={statistics.mean(numeric('matches')):.1f}"
        )
    else:
        print("tracking_diagnostics=not_available_for_profile")
    print(
        f"gpu_mean={statistics.mean(resources.gpu):.1f}% "
        f"median={statistics.median(resources.gpu):.1f}% "
        f"p90={percentile(resources.gpu, 90):.1f}% "
        f"max={max(resources.gpu):.0f}% "
        f"ge90={sum(value >= 90 for value in resources.gpu) / len(resources.gpu) * 100.0:.1f}%"
    )
    print(
        f"power_mean_w={statistics.mean(resources.power):.2f} "
        f"median={statistics.median(resources.power):.2f} "
        f"ram_mean_gib={statistics.mean(resources.ram):.2f}"
    )
    for module in MODULES:
        if resources.cpu[module]:
            print(
                f"module={module} cpu_cores={statistics.mean(resources.cpu[module]):.3f} "
                f"cpu_p90={percentile(resources.cpu[module], 90):.3f} "
                f"cpu_max={max(resources.cpu[module]):.3f} "
                f"rss_gib={statistics.mean(resources.rss[module]):.3f}"
            )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
