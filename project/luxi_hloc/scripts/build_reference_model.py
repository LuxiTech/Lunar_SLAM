#!/usr/bin/env python3
"""Build NetVLAD, SuperPoint and metric landmark files from an RTAB export."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
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
sys.path[:0] = [str(dependency) for dependency in dependencies]
os.environ.setdefault("TORCH_HOME", str(WORKSPACE / "3parts/hloc_models"))

import cv2  # noqa: E402
import h5py  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
import yaml  # noqa: E402
from luxi_hloc.geometry import transform_points  # noqa: E402
from luxi_hloc.inference import HlocFeatureBackend  # noqa: E402
from luxi_hloc.map_io import read_frames, sha256_file, validate_export  # noqa: E402
from luxi_hloc.pose_estimator import sample_depth_meters  # noqa: E402

if RUNTIME == "gpu" and not torch.cuda.is_available():
    raise RuntimeError(
        "LUXI_HLOC_RUNTIME=gpu requested but the installed PyTorch runtime "
        "cannot access CUDA"
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-directory", required=True, type=Path)
    parser.add_argument("--source-database", required=True, type=Path)
    parser.add_argument("--resize-max", type=int, default=640)
    parser.add_argument("--max-keypoints", type=int, default=1024)
    parser.add_argument("--minimum-depth", type=float, default=0.25)
    parser.add_argument("--maximum-depth", type=float, default=6.0)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def build_landmarks(
    map_directory: Path,
    local_features: Path,
    output_path: Path,
    minimum_depth: float,
    maximum_depth: float,
) -> dict:
    frames = read_frames(map_directory / "frames.csv")
    valid_total = 0
    keypoint_total = 0
    with h5py.File(local_features, "r") as features, h5py.File(output_path, "w") as output:
        for name, frame in frames.items():
            keypoints = np.asarray(features[name]["keypoints"], dtype=np.float64)
            depth = cv2.imread(
                str(map_directory / frame.depth_name),
                cv2.IMREAD_UNCHANGED,
            )
            if depth is None or depth.ndim != 2:
                raise RuntimeError(f"unable to read depth image for {name}")
            depth_meters = sample_depth_meters(depth, keypoints, frame.depth_scale)
            valid = (
                np.isfinite(depth_meters)
                & (depth_meters >= minimum_depth)
                & (depth_meters <= maximum_depth)
            )
            camera_points = np.zeros((len(keypoints), 3), dtype=np.float64)
            camera_points[:, 2] = np.nan_to_num(depth_meters, nan=0.0)
            camera_points[:, 0] = (
                (keypoints[:, 0] - frame.cx) * camera_points[:, 2] / frame.fx
            )
            camera_points[:, 1] = (
                (keypoints[:, 1] - frame.cy) * camera_points[:, 2] / frame.fy
            )
            map_points = transform_points(frame.map_from_camera, camera_points)
            map_points[~valid] = np.nan
            group = output.create_group(name)
            group.create_dataset("points3d", data=map_points.astype(np.float32))
            group.create_dataset("valid", data=valid.astype(np.uint8))
            group.create_dataset("depth_meters", data=depth_meters.astype(np.float32))
            keypoint_total += len(keypoints)
            valid_total += int(np.count_nonzero(valid))
    return {
        "keypoint_count": keypoint_total,
        "valid_landmark_count": valid_total,
        "valid_landmark_ratio": valid_total / max(1, keypoint_total),
    }


def extract_reference_features(
    map_directory: Path,
    image_names: list[str],
    retrieval_path: Path,
    local_path: Path,
    resize_max: int,
    max_keypoints: int,
) -> str:
    backend = HlocFeatureBackend("auto", resize_max, max_keypoints, cpu_threads=4)
    with h5py.File(retrieval_path, "w") as retrieval_file, h5py.File(local_path, "w") as local_file:
        for index, name in enumerate(image_names, start=1):
            bgr = cv2.imread(str(map_directory / name), cv2.IMREAD_COLOR)
            if bgr is None:
                raise RuntimeError(f"unable to read reference image {name}")
            descriptor, local = backend.extract(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
            retrieval_group = retrieval_file.create_group(name)
            retrieval_group.create_dataset(
                "global_descriptor",
                data=descriptor.astype(np.float16),
            )
            local_group = local_file.create_group(name)
            for key, value in local.items():
                array = np.asarray(value)
                if array.dtype == np.float32 and key != "image_size":
                    array = array.astype(np.float16)
                local_group.create_dataset(key, data=array)
            print(f"[{index}/{len(image_names)}] extracted {name}", flush=True)
    return backend.device


def main() -> int:
    arguments = parse_arguments()
    map_directory = arguments.map_directory.resolve()
    database = arguments.source_database.resolve()
    if not database.is_file():
        raise FileNotFoundError(database)
    frames = read_frames(map_directory / "frames.csv")
    validate_export(map_directory, frames.values())
    image_names = list(frames)

    retrieval_path = map_directory / "global-feats-netvlad.h5"
    local_path = map_directory / f"feats-superpoint-n{arguments.max_keypoints}.h5"
    landmark_path = map_directory / f"landmarks-superpoint-n{arguments.max_keypoints}.h5"

    started = time.perf_counter()
    existing_features = retrieval_path.exists() and local_path.exists()
    if (retrieval_path.exists() or local_path.exists()) and not existing_features:
        raise FileExistsError("only one feature file exists; pass --overwrite to rebuild")
    if existing_features and not arguments.overwrite:
        with h5py.File(retrieval_path, "r") as retrieval_file, h5py.File(
            local_path, "r"
        ) as local_file:
            if not all(name in retrieval_file and name in local_file for name in image_names):
                raise RuntimeError("feature files are incomplete; pass --overwrite to rebuild")
        build_device = "cuda" if torch.cuda.is_available() else "cpu"
        print("Reusing complete reference feature files.", flush=True)
    else:
        build_device = extract_reference_features(
            map_directory,
            image_names,
            retrieval_path,
            local_path,
            arguments.resize_max,
            arguments.max_keypoints,
        )
    landmark_report = build_landmarks(
        map_directory,
        local_path,
        landmark_path,
        arguments.minimum_depth,
        arguments.maximum_depth,
    )
    elapsed = time.perf_counter() - started
    metadata = {
        "schema_version": 1,
        "map_id": map_directory.name,
        "source_database": str(database),
        "source_database_sha256": sha256_file(database),
        "map_frame": "map",
        "camera_pose_convention": "T_map_camera",
        "image_count": len(frames),
        "reference_model": "netvlad",
        "local_feature_model": "superpoint",
        "matcher_model": "lightglue",
        "pose_solver": "opencv_solvePnPRansac",
        "retrieval_features": retrieval_path.name,
        "local_features": local_path.name,
        "landmarks": landmark_path.name,
        "resize_max": arguments.resize_max,
        "max_keypoints": arguments.max_keypoints,
        "hloc_version": "1.4",
        "hloc_commit": "80ccb7ee3bc048cb3a8ef221c5bc4d8ac25d5792",
        "lightglue_version": "0.2",
        "lightglue_commit": "edb2b838efb2ecfe3f88097c5fad9887d95aedad",
        "torch_version": str(torch.__version__),
        "build_device": build_device,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    with (map_directory / "metadata.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata, stream, sort_keys=False)
    report = {
        **metadata,
        **landmark_report,
        "elapsed_seconds": elapsed,
    }
    with (map_directory / "build_report.json").open("w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
