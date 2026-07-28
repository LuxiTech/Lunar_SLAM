#!/usr/bin/env python3
"""Run real HLoc inference against exported map frames and report pose errors."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import time


WORKSPACE = Path(__file__).resolve().parents[3]
RUNTIME = os.environ.get("LUXI_HLOC_RUNTIME", "auto").lower()
if RUNTIME not in {"auto", "gpu", "cpu"}:
    raise RuntimeError("LUXI_HLOC_RUNTIME must be one of: auto, gpu, cpu")
dependencies = [
    WORKSPACE / "3parts/hloc_python",
    WORKSPACE / "3parts/hloc",
    WORKSPACE / "3parts/lightglue",
    WORKSPACE / "install/luxi_hloc/local/lib/python3.10/dist-packages",
    WORKSPACE / "project/luxi_hloc",
]
gpu_runtime = WORKSPACE / "3parts/hloc_gpu_python"
if RUNTIME != "cpu" and gpu_runtime.is_dir():
    dependencies.insert(0, gpu_runtime)
elif RUNTIME == "gpu":
    raise RuntimeError(f"GPU runtime not found: {gpu_runtime}")
sys.path[:0] = [str(dependency) for dependency in dependencies]
os.environ.setdefault("TORCH_HOME", str(WORKSPACE / "3parts/hloc_models"))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

from luxi_hloc.geometry import camera_matrix  # noqa: E402
from luxi_hloc.inference import HlocLocalizer  # noqa: E402
from luxi_hloc.map_io import read_frames  # noqa: E402


def rotation_error_degrees(left: np.ndarray, right: np.ndarray) -> float:
    delta = left[:3, :3].T @ right[:3, :3]
    cosine = np.clip((np.trace(delta) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-directory", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--limit", type=int, default=5)
    parser.add_argument("--maximum-translation-error", type=float, default=0.10)
    parser.add_argument("--maximum-rotation-error-deg", type=float, default=10.0)
    parser.add_argument("--minimum-success-rate", type=float, default=0.8)
    parser.add_argument(
        "--include-self",
        action="store_true",
        help="Allow the query frame itself in retrieval; default tests neighboring frames.",
    )
    arguments = parser.parse_args()

    with arguments.config.open(encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream)["luxi_hloc_localizer"]["ros__parameters"]
    localizer = HlocLocalizer(arguments.map_directory, parameters)
    frames = read_frames(arguments.map_directory / "frames.csv")
    frame_values = list(frames.values())
    selected_indices = np.linspace(
        0,
        len(frame_values) - 1,
        min(arguments.limit, len(frame_values)),
        dtype=int,
    )
    selected = [frame_values[index] for index in selected_indices]
    results = []
    for frame in selected:
        bgr = cv2.imread(str(arguments.map_directory / frame.image_name), cv2.IMREAD_COLOR)
        depth = cv2.imread(
            str(arguments.map_directory / frame.depth_name),
            cv2.IMREAD_UNCHANGED,
        )
        if bgr is None or depth is None:
            raise RuntimeError(f"failed to read node {frame.node_id}")
        cpu_started = time.process_time()
        output = localizer.localize(
            cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB),
            depth,
            camera_matrix(frame.fx, frame.fy, frame.cx, frame.cy),
            frame.depth_scale,
            None if arguments.include_self else {frame.image_name},
        )
        cpu_seconds = time.process_time() - cpu_started
        translation_error = float("inf")
        rotation_error = float("inf")
        if output.map_from_camera is not None:
            translation_error = float(
                np.linalg.norm(output.map_from_camera[:3, 3] - frame.map_from_camera[:3, 3])
            )
            rotation_error = rotation_error_degrees(
                output.map_from_camera,
                frame.map_from_camera,
            )
        passed = (
            output.accepted
            and translation_error <= arguments.maximum_translation_error
            and rotation_error <= arguments.maximum_rotation_error_deg
        )
        results.append(
            {
                "node_id": frame.node_id,
                "passed": passed,
                "reason": output.reason,
                "reference": output.reference_name,
                "retrieval_score": output.retrieval_score,
                "translation_error": translation_error,
                "rotation_error_deg": rotation_error,
                "elapsed_seconds": output.elapsed_seconds,
                "cpu_seconds": cpu_seconds,
                "device": localizer.device,
            }
        )
        print(json.dumps(results[-1], ensure_ascii=False))
    success_rate = sum(item["passed"] for item in results) / max(1, len(results))
    summary = {
        "success_rate": success_rate,
        "passed": sum(item["passed"] for item in results),
        "tested": len(results),
        "device": localizer.device,
        "mean_elapsed_seconds": float(np.mean([item["elapsed_seconds"] for item in results])),
        "mean_cpu_seconds": float(np.mean([item["cpu_seconds"] for item in results])),
        "runtime_selection": RUNTIME,
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if success_rate >= arguments.minimum_success_rate else 1


if __name__ == "__main__":
    raise SystemExit(main())
