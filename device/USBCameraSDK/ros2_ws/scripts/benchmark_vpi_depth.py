#!/usr/bin/env python3
"""Compare USB VPI OFA stereo parameters on one stationary live scene."""

from __future__ import annotations

import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import time
import warnings

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


VARIANTS = (
    ("c64750_cpu_m5_40", 1, 64750, 7, True, 5, 0.04),
    ("c64750_cpu_m5_80", 1, 64750, 7, True, 5, 0.08),
    ("c64875_cpu_m5_40", 1, 64875, 7, True, 5, 0.04),
    ("c64875_cpu_m5_80", 1, 64875, 7, True, 5, 0.08),
    ("c64875_raw_m5_40", 1, 64875, 7, False, 5, 0.04),
    ("c64875_raw_m5_80", 1, 64875, 7, False, 5, 0.08),
    ("c65000_raw_m5_80", 1, 65000, 7, False, 5, 0.08),
)


def percentile(values: np.ndarray, level: float) -> float:
    """Return a percentile, or NaN when no samples survived."""
    return float(np.percentile(values, level)) if values.size else float("nan")


def summarize(name: str, frames: list[np.ndarray], arrivals: list[float]) -> dict:
    """Calculate temporal depth metrics after discarding cold-start frames."""
    frames = frames[-30:]
    arrivals = arrivals[-30:]
    depth = np.stack(frames).astype(np.float32)
    valid = depth > 0
    intervals = np.diff(arrivals)
    consecutive = []
    for first, second, first_valid, second_valid in zip(
        depth[:-1], depth[1:], valid[:-1], valid[1:]
    ):
        common = first_valid & second_valid
        if np.any(common):
            consecutive.append(np.abs(second[common] - first[common]))
    changes = np.concatenate(consecutive)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        nan_depth = np.where(valid, depth, np.nan)
        median = np.nanmedian(nan_depth, axis=0)
        mad = np.nanmedian(np.abs(nan_depth - median), axis=0)
    persistent = valid.sum(axis=0) >= math.ceil(0.8 * len(frames))
    stable = persistent & (mad <= 20)
    return {
        "name": name,
        "rate": 1.0 / statistics.mean(intervals),
        "valid": float(valid.mean() * 100),
        "delta50": percentile(changes, 50),
        "delta90": percentile(changes, 90),
        "delta99": percentile(changes, 99),
        "mad50": percentile(mad[persistent], 50),
        "mad90": percentile(mad[persistent], 90),
        "persistent": float(persistent.mean() * 100),
        "stable": float(stable.mean() * 100),
        "far": float(
            (valid & (depth >= 3000)).sum() / max(1, valid.sum()) * 100
        ),
    }


def main() -> None:
    """Run every fixed parameter variant against the already-running camera."""
    workspace = Path(__file__).resolve().parents[4]
    config = workspace / "device/USBCameraSDK/ros2_ws/src/usb_camera_driver/config/stereo_depth.yaml"
    calibration = workspace / "device/USBCameraSDK/ros2_ws/calibration/stereo_opencv.yaml"
    bridge = CvBridge()
    rclpy.init()
    node = rclpy.create_node("usb_vpi_depth_benchmark")
    frames: list[np.ndarray] = []
    arrivals: list[float] = []

    def receive(message: Image) -> None:
        if len(frames) >= 35:
            return
        frames.append(
            np.asarray(
                bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            ).copy()
        )
        arrivals.append(time.monotonic())

    node.create_subscription(
        Image, "/usb_stereo/depth", receive, qos_profile_sensor_data
    )
    results = []
    for name, passes, confidence, window, cpu_filter, output_median, output_delta in VARIANTS:
        frames.clear()
        arrivals.clear()
        command = [
            "ros2", "run", "usb_camera_driver", "stereo_depth_node",
            "--ros-args", "--params-file", str(config),
            "-p", f"calibration_file:={calibration}",
            "-p", f"vpi_confidence_threshold:={confidence}",
            "-p", f"vpi_ofa_num_passes:={passes}",
            "-p", f"vpi_ofa_window_size:={window}",
            "-p", f"vpi_ofa_apply_cpu_postfilters:={'true' if cpu_filter else 'false'}",
            "-p", f"vpi_output_median_filter_size:={output_median}",
            "-p", f"vpi_output_median_max_difference_m:={output_delta}",
        ]
        process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=os.environ.copy(),
        )
        deadline = time.monotonic() + 15.0
        try:
            while len(frames) < 35 and time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f"{name} depth node exited early")
                rclpy.spin_once(node, timeout_sec=0.2)
            if len(frames) < 35:
                raise RuntimeError(f"{name} received only {len(frames)} frames")
            result = summarize(name, frames, arrivals)
            results.append(result)
            print(
                "{name:20s} rate={rate:5.2f} valid={valid:5.2f}% "
                "delta50/90={delta50:5.1f}/{delta90:5.1f}mm "
                "mad50/90={mad50:4.1f}/{mad90:4.1f}mm "
                "stable={stable:5.2f}% far={far:5.2f}%".format(**result),
                flush=True,
            )
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=2.0)
            time.sleep(0.5)

    print("\nCSV")
    keys = tuple(results[0])
    print(",".join(keys))
    for result in results:
        print(",".join(str(result[key]) for key in keys))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
