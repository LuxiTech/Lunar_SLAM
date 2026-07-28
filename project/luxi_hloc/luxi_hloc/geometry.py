"""Geometry utilities with explicit transform direction conventions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class PnpResult:
    """Pose returned by PnP.

    ``camera_from_map`` transforms map-frame points into the query camera frame.
    """

    camera_from_map: np.ndarray
    inlier_indices: np.ndarray
    reprojection_rmse: float


def invert_transform(transform: np.ndarray) -> np.ndarray:
    """Invert a rigid 4x4 transform."""
    transform = np.asarray(transform, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError("transform must have shape (4, 4)")
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = transform[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ transform[:3, 3]
    return inverse


def transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Transform Nx3 points with a 4x4 rigid transform."""
    transform = np.asarray(transform, dtype=np.float64)
    points = np.asarray(points, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError("transform must have shape (4, 4)")
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    return points @ transform[:3, :3].T + transform[:3, 3]


def camera_matrix(fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    """Create an OpenCV pinhole camera matrix."""
    values = (fx, fy, cx, cy)
    if not all(np.isfinite(value) for value in values) or fx <= 0.0 or fy <= 0.0:
        raise ValueError("camera intrinsics are invalid")
    return np.array(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def solve_pnp_ransac(
    map_points: np.ndarray,
    image_points: np.ndarray,
    intrinsics: np.ndarray,
    reprojection_error: float,
    iterations: int,
    confidence: float,
) -> Optional[PnpResult]:
    """Estimate ``T_camera_map`` using OpenCV PnP/RANSAC and LM refinement."""
    map_points = np.asarray(map_points, dtype=np.float64)
    image_points = np.asarray(image_points, dtype=np.float64)
    intrinsics = np.asarray(intrinsics, dtype=np.float64)
    if map_points.ndim != 2 or map_points.shape[1] != 3:
        raise ValueError("map_points must have shape (N, 3)")
    if image_points.shape != (map_points.shape[0], 2):
        raise ValueError("image_points must have shape (N, 2)")
    if intrinsics.shape != (3, 3):
        raise ValueError("intrinsics must have shape (3, 3)")
    if map_points.shape[0] < 4:
        return None

    success, rotation_vector, translation, inliers = cv2.solvePnPRansac(
        objectPoints=map_points,
        imagePoints=image_points,
        cameraMatrix=intrinsics,
        distCoeffs=np.zeros((4, 1), dtype=np.float64),
        iterationsCount=iterations,
        reprojectionError=reprojection_error,
        confidence=confidence,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if not success or inliers is None or len(inliers) < 4:
        return None

    inlier_indices = inliers.reshape(-1)
    rotation_vector, translation = cv2.solvePnPRefineLM(
        map_points[inlier_indices],
        image_points[inlier_indices],
        intrinsics,
        np.zeros((4, 1), dtype=np.float64),
        rotation_vector,
        translation,
    )
    rotation, _ = cv2.Rodrigues(rotation_vector)
    camera_from_map = np.eye(4, dtype=np.float64)
    camera_from_map[:3, :3] = rotation
    camera_from_map[:3, 3] = translation.reshape(3)

    projected, _ = cv2.projectPoints(
        map_points[inlier_indices],
        rotation_vector,
        translation,
        intrinsics,
        np.zeros((4, 1), dtype=np.float64),
    )
    residuals = projected.reshape(-1, 2) - image_points[inlier_indices]
    rmse = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))
    return PnpResult(camera_from_map, inlier_indices, rmse)


def quaternion_from_rotation(rotation: np.ndarray) -> np.ndarray:
    """Return ROS-order quaternion (x, y, z, w) from a 3x3 rotation."""
    rotation = np.asarray(rotation, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError("rotation must have shape (3, 3)")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale,
            ]
        )
    else:
        diagonal = int(np.argmax(np.diag(rotation)))
        if diagonal == 0:
            scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                ]
            )
        elif diagonal == 1:
            scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                ]
            )
        else:
            scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quaternion = np.array(
                [
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                ]
            )
    return quaternion / np.linalg.norm(quaternion)
