"""PnP candidate validation for HLoc matches."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import PnpResult, solve_pnp_ransac


@dataclass(frozen=True)
class CandidatePose:
    accepted: bool
    reason: str
    pnp: PnpResult | None
    match_count: int
    valid_landmark_count: int
    inlier_count: int
    inlier_ratio: float
    depth_verified_count: int
    median_depth_residual: float


def sample_depth_meters(
    depth: np.ndarray,
    points: np.ndarray,
    depth_scale: float,
    radius: int = 1,
) -> np.ndarray:
    """Sample robust median depth around image points."""
    if depth.ndim != 2:
        raise ValueError("depth image must be single-channel")
    values = np.full(len(points), np.nan, dtype=np.float64)
    height, width = depth.shape
    for index, (x, y) in enumerate(np.asarray(points)):
        column = int(round(float(x)))
        row = int(round(float(y)))
        if column < 0 or row < 0 or column >= width or row >= height:
            continue
        x0 = max(0, column - radius)
        x1 = min(width, column + radius + 1)
        y0 = max(0, row - radius)
        y1 = min(height, row + radius + 1)
        patch = depth[y0:y1, x0:x1].astype(np.float64) * depth_scale
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if valid.size:
            values[index] = float(np.median(valid))
    return values


def estimate_candidate(
    map_points: np.ndarray,
    query_points: np.ndarray,
    intrinsics: np.ndarray,
    query_depth: np.ndarray | None,
    query_depth_scale: float,
    *,
    match_count: int,
    minimum_matches: int,
    minimum_landmarks: int,
    minimum_inliers: int,
    minimum_inlier_ratio: float,
    maximum_reprojection_rmse: float,
    ransac_reprojection_error: float,
    ransac_iterations: int,
    ransac_confidence: float,
    require_depth_verification: bool,
    minimum_depth_verified: int,
    maximum_median_depth_residual: float,
    minimum_depth: float,
    maximum_depth: float,
) -> CandidatePose:
    """Run PnP and reject weak or depth-inconsistent solutions."""

    def rejected(reason: str, pnp: PnpResult | None = None) -> CandidatePose:
        inlier_count = 0 if pnp is None else len(pnp.inlier_indices)
        ratio = inlier_count / max(1, len(map_points))
        return CandidatePose(
            False,
            reason,
            pnp,
            match_count,
            len(map_points),
            inlier_count,
            ratio,
            0,
            float("inf"),
        )

    if match_count < minimum_matches:
        return rejected("MATCHES_LOW")
    if len(map_points) < minimum_landmarks:
        return rejected("LANDMARKS_LOW")
    pnp = solve_pnp_ransac(
        map_points,
        query_points,
        intrinsics,
        ransac_reprojection_error,
        ransac_iterations,
        ransac_confidence,
    )
    if pnp is None:
        return rejected("PNP_FAILED")
    inlier_count = len(pnp.inlier_indices)
    inlier_ratio = inlier_count / max(1, len(map_points))
    if inlier_count < minimum_inliers:
        return rejected("PNP_INLIERS_LOW", pnp)
    if inlier_ratio < minimum_inlier_ratio:
        return rejected("PNP_INLIER_RATIO_LOW", pnp)
    if pnp.reprojection_rmse > maximum_reprojection_rmse:
        return rejected("REPROJECTION_ERROR_HIGH", pnp)

    if not require_depth_verification:
        return CandidatePose(
            True,
            "ACCEPTED",
            pnp,
            match_count,
            len(map_points),
            inlier_count,
            inlier_ratio,
            0,
            0.0,
        )
    if query_depth is None:
        return rejected("DEPTH_MISSING", pnp)

    inliers = pnp.inlier_indices
    measured_depth = sample_depth_meters(query_depth, query_points[inliers], query_depth_scale)
    rotation = pnp.camera_from_map[:3, :3]
    translation = pnp.camera_from_map[:3, 3]
    predicted_depth = (map_points[inliers] @ rotation.T + translation)[:, 2]
    valid = (
        np.isfinite(measured_depth)
        & np.isfinite(predicted_depth)
        & (measured_depth >= minimum_depth)
        & (measured_depth <= maximum_depth)
        & (predicted_depth > 0.0)
    )
    verified_count = int(np.count_nonzero(valid))
    if verified_count < minimum_depth_verified:
        result = rejected("DEPTH_VALID_POINTS_LOW", pnp)
        return CandidatePose(**{**result.__dict__, "depth_verified_count": verified_count})
    median_residual = float(np.median(np.abs(measured_depth[valid] - predicted_depth[valid])))
    if median_residual > maximum_median_depth_residual:
        result = rejected("DEPTH_RESIDUAL_HIGH", pnp)
        return CandidatePose(
            **{
                **result.__dict__,
                "depth_verified_count": verified_count,
                "median_depth_residual": median_residual,
            }
        )
    return CandidatePose(
        True,
        "ACCEPTED",
        pnp,
        match_count,
        len(map_points),
        inlier_count,
        inlier_ratio,
        verified_count,
        median_residual,
    )
