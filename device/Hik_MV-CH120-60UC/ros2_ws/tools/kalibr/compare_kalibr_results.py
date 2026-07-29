#!/usr/bin/env python3

"""Compare repeated Kalibr camera-IMU results.

Usage:
  python3 compare_kalibr_results.py result1.yaml result2.yaml [result3.yaml ...]
"""

import itertools
import math
import pathlib
import sys

import numpy as np
import yaml


def load_result(path_string):
    path = pathlib.Path(path_string)
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    cam0 = data.get("cam0", {})
    matrix = np.asarray(cam0.get("T_cam_imu"), dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"{path}: cam0.T_cam_imu is missing or is not 4x4")

    shift = float(cam0.get("timeshift_cam_imu", 0.0))
    return path, matrix, shift


def rotation_difference_degrees(rotation_a, rotation_b):
    relative = rotation_a @ rotation_b.T
    cosine = (np.trace(relative) - 1.0) / 2.0
    cosine = float(np.clip(cosine, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def main():
    if len(sys.argv) < 3:
        print(__doc__.strip(), file=sys.stderr)
        return 2

    results = [load_result(argument) for argument in sys.argv[1:]]
    print("Kalibr repeated-run comparison (cam0 = left camera)")
    for path, matrix, shift in results:
        translation = matrix[:3, 3]
        print(
            f"- {path}: t=[{translation[0]:.6f}, {translation[1]:.6f}, "
            f"{translation[2]:.6f}] m, shift={shift * 1000.0:.3f} ms"
        )

    print("\nPairwise differences")
    for result_a, result_b in itertools.combinations(results, 2):
        path_a, matrix_a, shift_a = result_a
        path_b, matrix_b, shift_b = result_b
        translation_mm = np.linalg.norm(matrix_a[:3, 3] - matrix_b[:3, 3]) * 1000.0
        rotation_deg = rotation_difference_degrees(matrix_a[:3, :3], matrix_b[:3, :3])
        shift_ms = abs(shift_a - shift_b) * 1000.0
        print(
            f"- {path_a.name} vs {path_b.name}: "
            f"translation={translation_mm:.3f} mm, "
            f"rotation={rotation_deg:.3f} deg, "
            f"time shift={shift_ms:.3f} ms"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
