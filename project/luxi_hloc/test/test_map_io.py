import csv
from pathlib import Path

import numpy as np

from luxi_hloc.map_io import MATRIX_COLUMNS, read_frames, validate_export


def test_read_and_validate_export(tmp_path: Path):
    (tmp_path / "reference_images").mkdir()
    (tmp_path / "reference_depth").mkdir()
    (tmp_path / "reference_images/node.jpg").write_bytes(b"image")
    (tmp_path / "reference_depth/node.png").write_bytes(b"depth")
    header = [
        "node_id", "image_name", "depth_name", "stamp", "map_id",
        "fx", "fy", "cx", "cy", "width", "height", "depth_scale", *MATRIX_COLUMNS
    ]
    matrix = np.eye(4).reshape(-1)
    with (tmp_path / "frames.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerow([
            7, "reference_images/node.jpg", "reference_depth/node.png", 1.5, 0,
            500.0, 501.0, 320.0, 240.0, 640, 480, 0.001, *matrix
        ])
    frames = read_frames(tmp_path / "frames.csv")
    assert frames["reference_images/node.jpg"].node_id == 7
    validate_export(tmp_path, frames.values())
