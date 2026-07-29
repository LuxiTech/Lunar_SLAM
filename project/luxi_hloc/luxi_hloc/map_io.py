"""Read and validate RTAB-to-HLoc map artifacts."""

from __future__ import annotations

import csv
import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable

import numpy as np
import yaml


MATRIX_COLUMNS = tuple(f"t{row}{column}" for row in range(4) for column in range(4))


@dataclass(frozen=True)
class ReferenceFrame:
    node_id: int
    image_name: str
    depth_name: str
    stamp: float
    map_id: int
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    depth_scale: float
    map_from_camera: np.ndarray


def sha256_file(path: Path, chunk_size: int = 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(chunk_size):
            digest.update(chunk)
    return digest.hexdigest()


def read_frames(path: Path) -> Dict[str, ReferenceFrame]:
    """Read exporter CSV keyed by relative reference image name."""
    frames: Dict[str, ReferenceFrame] = {}
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {
            "node_id",
            "image_name",
            "depth_name",
            "stamp",
            "map_id",
            "fx",
            "fy",
            "cx",
            "cy",
            "width",
            "height",
            "depth_scale",
            *MATRIX_COLUMNS,
        }
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(f"frames CSV is missing columns: {sorted(missing)}")
        for row in reader:
            matrix = np.array([float(row[name]) for name in MATRIX_COLUMNS]).reshape(4, 4)
            image_name = row["image_name"]
            if image_name in frames:
                raise ValueError(f"duplicate reference image: {image_name}")
            frames[image_name] = ReferenceFrame(
                node_id=int(row["node_id"]),
                image_name=image_name,
                depth_name=row["depth_name"],
                stamp=float(row["stamp"]),
                map_id=int(row["map_id"]),
                fx=float(row["fx"]),
                fy=float(row["fy"]),
                cx=float(row["cx"]),
                cy=float(row["cy"]),
                width=int(row["width"]),
                height=int(row["height"]),
                depth_scale=float(row["depth_scale"]),
                map_from_camera=matrix,
            )
    if not frames:
        raise ValueError(f"no frames found in {path}")
    return frames


def validate_export(export_directory: Path, frames: Iterable[ReferenceFrame]) -> None:
    for frame in frames:
        image_path = export_directory / frame.image_name
        depth_path = export_directory / frame.depth_name
        if not image_path.is_file():
            raise FileNotFoundError(image_path)
        if not depth_path.is_file():
            raise FileNotFoundError(depth_path)
        if frame.width <= 0 or frame.height <= 0:
            raise ValueError(f"invalid dimensions for node {frame.node_id}")
        if frame.fx <= 0.0 or frame.fy <= 0.0:
            raise ValueError(f"invalid intrinsics for node {frame.node_id}")
        if frame.map_from_camera.shape != (4, 4):
            raise ValueError(f"invalid pose for node {frame.node_id}")
        if not np.allclose(frame.map_from_camera[3], (0.0, 0.0, 0.0, 1.0)):
            raise ValueError(f"invalid homogeneous pose for node {frame.node_id}")


def read_metadata(map_directory: Path) -> dict:
    path = map_directory / "metadata.yaml"
    if not path.is_file():
        raise FileNotFoundError(path)
    with path.open(encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    if not isinstance(metadata, dict) or metadata.get("schema_version") != 1:
        raise ValueError("unsupported or missing HLoc map schema_version")
    required_files = (
        "frames.csv",
        metadata.get("retrieval_features", ""),
        metadata.get("local_features", ""),
        metadata.get("landmarks", ""),
    )
    for name in required_files:
        if not name or not (map_directory / name).is_file():
            raise FileNotFoundError(map_directory / name)
    return metadata
