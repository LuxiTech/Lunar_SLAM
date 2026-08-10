"""Depth projection and rigid pose estimation without ROS dependencies."""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class RelativePose:
    """Geometrically verified transform from reference to current camera."""

    current_from_reference: np.ndarray
    inlier_indices: np.ndarray
    reprojection_rmse: float


def invert_transform(transform: np.ndarray) -> np.ndarray:
    """Invert a finite 4x4 rigid transform."""
    matrix = np.asarray(transform, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("transform must be a finite 4x4 matrix")
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = matrix[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ matrix[:3, 3]
    return inverse


def rotation_angle(transform: np.ndarray) -> float:
    """Return the absolute rotation angle of a rigid transform in radians."""
    rotation = np.asarray(transform, dtype=np.float64)[:3, :3]
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    return math.acos(cosine)


def project_depth_points(
    keypoints: np.ndarray,
    depth: np.ndarray,
    intrinsics: np.ndarray,
    depth_scale: float,
    minimum_depth: float,
    maximum_depth: float,
    sampling_radius: int = 0,
    minimum_valid_samples: int = 1,
) -> tuple[np.ndarray, np.ndarray]:
    """Project image keypoints to camera 3D; invalid rows remain NaN.

    A non-zero ``sampling_radius`` uses the median of valid samples around a
    keypoint.  Learned keypoints often land on a stereo pixel whose single
    disparity estimate is noisy or missing; a small local median is much more
    stable while still preserving foreground/background boundaries.
    """
    pixels = np.asarray(keypoints, dtype=np.float64)
    image = np.asarray(depth)
    camera = np.asarray(intrinsics, dtype=np.float64)
    if pixels.ndim != 2 or pixels.shape[1] != 2:
        raise ValueError("keypoints must be Nx2")
    if image.ndim != 2:
        raise ValueError("depth must be single-channel")
    if camera.shape != (3, 3):
        raise ValueError("intrinsics must be 3x3")
    if depth_scale <= 0.0 or minimum_depth < 0.0 or maximum_depth <= minimum_depth:
        raise ValueError("invalid depth limits")
    if sampling_radius < 0 or minimum_valid_samples < 1:
        raise ValueError("invalid depth sampling parameters")

    rounded = np.rint(pixels).astype(np.int64)
    valid_pixels = (
        (rounded[:, 0] >= 0)
        & (rounded[:, 0] < image.shape[1])
        & (rounded[:, 1] >= 0)
        & (rounded[:, 1] < image.shape[0])
    )
    metric_depth = np.full(len(pixels), np.nan, dtype=np.float64)
    indexes = np.flatnonzero(valid_pixels)
    if len(indexes) and sampling_radius == 0:
        metric_depth[indexes] = (
            image[rounded[indexes, 1], rounded[indexes, 0]].astype(np.float64)
            * depth_scale
        )
    elif len(indexes):
        # Gather every keypoint patch in one small NxK array. This avoids a
        # Python/NumPy allocation for each SuperPoint keypoint and keeps robust
        # sampling below the neural frontend's frame budget on Jetson.
        offset_y, offset_x = np.mgrid[
            -sampling_radius:sampling_radius + 1,
            -sampling_radius:sampling_radius + 1,
        ]
        sample_x = rounded[indexes, 0, None] + offset_x.reshape(1, -1)
        sample_y = rounded[indexes, 1, None] + offset_y.reshape(1, -1)
        inside = (
            (sample_x >= 0)
            & (sample_x < image.shape[1])
            & (sample_y >= 0)
            & (sample_y < image.shape[0])
        )
        samples = image[
            np.clip(sample_y, 0, image.shape[0] - 1),
            np.clip(sample_x, 0, image.shape[1] - 1),
        ].astype(np.float64)
        samples *= depth_scale
        usable = (
            inside
            & np.isfinite(samples)
            & (samples >= minimum_depth)
            & (samples <= maximum_depth)
        )
        enough = usable.sum(axis=1) >= minimum_valid_samples
        if np.any(enough):
            robust_depth = np.nanmedian(
                np.where(usable[enough], samples[enough], np.nan), axis=1
            )
            metric_depth[indexes[enough]] = robust_depth
    valid = (
        valid_pixels
        & np.isfinite(metric_depth)
        & (metric_depth >= minimum_depth)
        & (metric_depth <= maximum_depth)
    )
    points = np.full((len(pixels), 3), np.nan, dtype=np.float32)
    z = metric_depth[valid]
    points[valid, 0] = ((pixels[valid, 0] - camera[0, 2]) * z / camera[0, 0]).astype(
        np.float32
    )
    points[valid, 1] = ((pixels[valid, 1] - camera[1, 2]) * z / camera[1, 1]).astype(
        np.float32
    )
    points[valid, 2] = z.astype(np.float32)
    return points, valid


def estimate_relative_pose(
    reference_points: np.ndarray,
    current_pixels: np.ndarray,
    intrinsics: np.ndarray,
    reprojection_error: float,
    iterations: int,
    confidence: float,
) -> RelativePose | None:
    """Estimate ``T_current_camera_reference_camera`` with PnP/RANSAC."""
    points3d = np.asarray(reference_points, dtype=np.float64)
    points2d = np.asarray(current_pixels, dtype=np.float64)
    camera = np.asarray(intrinsics, dtype=np.float64)
    if points3d.ndim != 2 or points3d.shape[1] != 3:
        raise ValueError("reference_points must be Nx3")
    if points2d.shape != (len(points3d), 2):
        raise ValueError("current_pixels must correspond to reference_points")
    if len(points3d) < 4:
        return None
    success, rotation_vector, translation, inliers = cv2.solvePnPRansac(
        points3d,
        points2d,
        camera,
        None,
        iterationsCount=int(iterations),
        reprojectionError=float(reprojection_error),
        confidence=float(confidence),
        flags=cv2.SOLVEPNP_EPNP,
    )
    if not success or inliers is None or len(inliers) < 4:
        return None
    inlier_indices = inliers.reshape(-1).astype(np.int64)
    cv2.solvePnP(
        points3d[inlier_indices],
        points2d[inlier_indices],
        camera,
        None,
        rotation_vector,
        translation,
        True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    rotation, _ = cv2.Rodrigues(rotation_vector)
    current_from_reference = np.eye(4, dtype=np.float64)
    current_from_reference[:3, :3] = rotation
    current_from_reference[:3, 3] = translation.reshape(3)
    projected, _ = cv2.projectPoints(
        points3d[inlier_indices], rotation_vector, translation, camera, None
    )
    residuals = projected.reshape(-1, 2) - points2d[inlier_indices]
    rmse = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))
    if not np.all(np.isfinite(current_from_reference)) or not math.isfinite(rmse):
        return None
    return RelativePose(current_from_reference, inlier_indices, rmse)


def estimate_translation_with_rotation(
    reference_points: np.ndarray,
    current_points: np.ndarray,
    current_pixels: np.ndarray,
    intrinsics: np.ndarray,
    current_from_reference_rotation: np.ndarray,
    maximum_3d_error: float,
    minimum_inliers: int,
) -> RelativePose | None:
    """Estimate translation robustly while keeping an external rotation fixed."""
    reference = np.asarray(reference_points, dtype=np.float64)
    current = np.asarray(current_points, dtype=np.float64)
    pixels = np.asarray(current_pixels, dtype=np.float64)
    camera = np.asarray(intrinsics, dtype=np.float64)
    rotation = np.asarray(current_from_reference_rotation, dtype=np.float64)
    if reference.ndim != 2 or reference.shape[1] != 3:
        raise ValueError("reference_points must be Nx3")
    if current.shape != reference.shape or pixels.shape != (len(reference), 2):
        raise ValueError("current points and pixels must correspond to reference points")
    if camera.shape != (3, 3) or rotation.shape != (3, 3):
        raise ValueError("intrinsics and rotation must be 3x3")
    if maximum_3d_error <= 0.0 or minimum_inliers < 3:
        raise ValueError("invalid fixed-rotation estimator thresholds")
    finite = (
        np.all(np.isfinite(reference), axis=1)
        & np.all(np.isfinite(current), axis=1)
        & np.all(np.isfinite(pixels), axis=1)
    )
    finite_indices = np.flatnonzero(finite)
    if len(finite_indices) < minimum_inliers:
        return None

    translations = current[finite] - reference[finite] @ rotation.T
    translation = np.median(translations, axis=0)
    residuals_3d = np.linalg.norm(translations - translation, axis=1)
    consistent = residuals_3d <= maximum_3d_error
    if np.count_nonzero(consistent) < minimum_inliers:
        return None
    translation = np.median(translations[consistent], axis=0)
    residuals_3d = np.linalg.norm(translations - translation, axis=1)
    consistent = residuals_3d <= maximum_3d_error
    if np.count_nonzero(consistent) < minimum_inliers:
        return None

    inlier_indices = finite_indices[consistent]
    rotation_vector, _ = cv2.Rodrigues(rotation)
    projected, _ = cv2.projectPoints(
        reference[inlier_indices],
        rotation_vector,
        translation.reshape(3, 1),
        camera,
        None,
    )
    residuals_2d = projected.reshape(-1, 2) - pixels[inlier_indices]
    rmse = float(np.sqrt(np.mean(np.sum(residuals_2d * residuals_2d, axis=1))))
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    if not np.all(np.isfinite(transform)) or not math.isfinite(rmse):
        return None
    return RelativePose(transform, inlier_indices, rmse)


def image_grid_coverage(
    points: np.ndarray,
    image_width: int,
    image_height: int,
    columns: int = 4,
    rows: int = 3,
) -> float:
    """Return the occupied fraction of a fixed image grid."""
    pixels = np.asarray(points, dtype=np.float64)
    if len(pixels) == 0 or image_width <= 0 or image_height <= 0:
        return 0.0
    x = np.clip((pixels[:, 0] * columns / image_width).astype(int), 0, columns - 1)
    y = np.clip((pixels[:, 1] * rows / image_height).astype(int), 0, rows - 1)
    return len(set(zip(x.tolist(), y.tolist()))) / float(columns * rows)
