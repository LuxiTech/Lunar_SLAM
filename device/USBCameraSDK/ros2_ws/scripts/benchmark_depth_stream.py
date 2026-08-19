#!/usr/bin/env python3
"""Measure rate, coverage, temporal stability and Jetson load of a depth topic."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import re
import signal
import statistics
import subprocess
import threading
import time
import warnings

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def image_array(message: Image) -> np.ndarray:
    if message.encoding != "16UC1":
        raise RuntimeError(f"expected 16UC1, got {message.encoding}")
    return np.frombuffer(message.data, dtype=np.uint16).reshape(
        message.height, message.step // 2
    )[:, : message.width].copy()


class JetsonSampler:
    def __init__(self, process_pattern: str):
        self.process_pattern = process_pattern
        self.stop = threading.Event()
        self.gpu = []
        self.power = []
        self.ram = []
        self.cpu = []
        self.rss = []
        self.tegrastats = None
        self.thread = None

    def start(self):
        self.tegrastats = subprocess.Popen(
            ["tegrastats", "--interval", "250"], stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True, start_new_session=True,
        )
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _matching_pids(self):
        result = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode()
            except (OSError, UnicodeDecodeError):
                continue
            if self.process_pattern in command and "benchmark_depth_stream.py" not in command:
                result.append(int(entry.name))
        return result

    def _run(self):
        previous = {}
        previous_time = time.monotonic()
        clock_ticks = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
        page_size = os.sysconf("SC_PAGE_SIZE")
        assert self.tegrastats is not None and self.tegrastats.stdout is not None
        for line in self.tegrastats.stdout:
            gpu = re.search(r"GR3D_FREQ (\d+)%", line)
            power = re.search(r"VDD_IN (\d+)mW", line)
            ram = re.search(r"RAM (\d+)/", line)
            if gpu:
                self.gpu.append(float(gpu.group(1)))
            if power:
                self.power.append(float(power.group(1)) / 1000.0)
            if ram:
                self.ram.append(float(ram.group(1)) / 1024.0)
            now = time.monotonic()
            current = {}
            rss = 0.0
            for pid in self._matching_pids():
                try:
                    fields = Path(f"/proc/{pid}/stat").read_text().split()
                    current[pid] = int(fields[13]) + int(fields[14])
                    rss += int(fields[23]) * page_size / 1024.0**3
                except (OSError, ValueError, IndexError):
                    pass
            elapsed = now - previous_time
            if previous and elapsed > 0.0:
                ticks = sum(current.get(pid, value) - value for pid, value in previous.items())
                self.cpu.append(ticks / clock_ticks / elapsed)
            if current:
                self.rss.append(rss)
            previous, previous_time = current, now
            if self.stop.is_set():
                break

    def finish(self):
        self.stop.set()
        if self.tegrastats is not None and self.tegrastats.poll() is None:
            os.killpg(self.tegrastats.pid, signal.SIGINT)
            self.tegrastats.wait(timeout=2.0)
        if self.thread:
            self.thread.join(timeout=2.0)


def percentile(values, level):
    return float(np.percentile(values, level)) if len(values) else float("nan")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/usb_stereo/depth")
    parser.add_argument("--process-pattern", default="stereo_depth_node")
    parser.add_argument("--warmup", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=20.0)
    arguments = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("usb_depth_stream_benchmark")
    frames, arrivals = [], []

    def receive(message):
        frames.append(image_array(message))
        arrivals.append(time.monotonic())

    node.create_subscription(Image, arguments.topic, receive, qos_profile_sensor_data)
    deadline = time.monotonic() + arguments.warmup
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    frames.clear()
    arrivals.clear()
    resources = JetsonSampler(arguments.process_pattern)
    resources.start()
    deadline = time.monotonic() + arguments.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    resources.finish()
    if len(frames) < 2:
        raise RuntimeError(f"received only {len(frames)} depth frames")
    depth = np.stack(frames).astype(np.float32)
    valid = depth > 0
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        nan_depth = np.where(valid, depth, np.nan)
        temporal_median = np.nanmedian(nan_depth, axis=0)
        temporal_mad = np.nanmedian(np.abs(nan_depth - temporal_median), axis=0)
    persistent = valid.sum(axis=0) >= math.ceil(0.8 * len(frames))
    changes = []
    for first, second in zip(depth[:-1], depth[1:]):
        common = (first > 0) & (second > 0)
        if np.any(common):
            changes.append(np.abs(first[common] - second[common]))
    changes = np.concatenate(changes) if changes else np.array([])
    intervals = np.diff(arrivals)
    valid_values = depth[valid]
    print(f"frames={len(frames)} rate_hz={1.0 / statistics.mean(intervals):.3f}")
    print(
        f"valid_pct={valid.mean() * 100:.3f} persistent_pct={persistent.mean() * 100:.3f} "
        f"far_valid_pct={np.count_nonzero(valid_values >= 3000) / valid_values.size * 100:.3f}"
    )
    print(
        f"depth_mm_p10/p50/p90={percentile(valid_values, 10):.1f}/"
        f"{percentile(valid_values, 50):.1f}/{percentile(valid_values, 90):.1f}"
    )
    print(
        f"delta_mm_p50/p90/p99={percentile(changes, 50):.1f}/"
        f"{percentile(changes, 90):.1f}/{percentile(changes, 99):.1f} "
        f"mad_mm_p50/p90={percentile(temporal_mad[persistent], 50):.1f}/"
        f"{percentile(temporal_mad[persistent], 90):.1f}"
    )
    print(
        f"gpu_pct_mean/p95={statistics.mean(resources.gpu):.2f}/"
        f"{percentile(resources.gpu, 95):.2f} power_w_mean/p95="
        f"{statistics.mean(resources.power):.3f}/{percentile(resources.power, 95):.3f} "
        f"ram_gib_mean={statistics.mean(resources.ram):.3f}"
    )
    if resources.cpu:
        print(
            f"process_cpu_cores_mean/p95={statistics.mean(resources.cpu):.3f}/"
            f"{percentile(resources.cpu, 95):.3f} process_rss_gib_mean="
            f"{statistics.mean(resources.rss):.3f}"
        )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
