"""Keyframe RGB-D visual odometry driven by a learned feature backend."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Protocol

import numpy as np

from .feature_backend import NeuralFeatures
from .geometry import (
    estimate_relative_pose,
    estimate_translation_with_rotation,
    image_grid_coverage,
    invert_transform,
    project_depth_points,
    rotation_angle,
)


class FeatureBackend(Protocol):
    """Feature operations required by the tracker."""

    def extract(self, rgb: np.ndarray) -> NeuralFeatures:
        """Extract learned keypoints and descriptors from an RGB image."""
        ...

    def match(
        self,
        first: NeuralFeatures,
        second: NeuralFeatures,
    ) -> np.ndarray:
        """Match first-image features to second-image features."""
        ...


@dataclass(frozen=True)
class TrackerConfig:
    """Tracking thresholds expressed in metric and pixel units."""

    minimum_keypoints: int = 80
    minimum_matches: int = 50
    minimum_depth_matches: int = 30
    minimum_inliers: int = 25
    minimum_inlier_ratio: float = 0.25
    minimum_grid_coverage: float = 0.25
    maximum_reprojection_rmse: float = 3.0
    ransac_reprojection_error: float = 4.0
    ransac_iterations: int = 1000
    ransac_confidence: float = 0.999
    minimum_depth: float = 0.2
    maximum_depth: float = 6.0
    keyframe_min_translation: float = 0.10
    keyframe_min_rotation: float = math.radians(8.0)
    keyframe_max_age: float = 1.0
    keyframe_min_inlier_ratio: float = 0.40
    maximum_frame_translation: float = 1.0
    maximum_frame_rotation: float = math.radians(60.0)
    maximum_consecutive_tracking_failures: int = 3
    minimum_depth_consistency_matches: int = 20
    maximum_depth_consistency_error: float = 0.08
    maximum_imu_rotation_error: float = math.radians(12.0)


@dataclass(frozen=True)
class TrackingResult:
    """One frontend update including the data sent to RTAB-Map."""

    accepted: bool
    state: str
    reason: str
    odom_from_camera: np.ndarray | None
    features: NeuralFeatures
    feature_points3d: np.ndarray
    keypoint_count: int
    match_count: int
    depth_match_count: int
    inlier_count: int
    inlier_ratio: float
    grid_coverage: float
    reprojection_rmse: float | None
    elapsed_seconds: float
    keyframe_updated: bool
    pose_source: str = "NONE"
    imu_rotation_error: float | None = None
    depth_consistency_inliers: int = 0


@dataclass
class _Keyframe:
    features: NeuralFeatures
    points3d: np.ndarray
    valid_depth: np.ndarray
    odom_from_camera: np.ndarray
    stamp: float
    world_from_camera_rotation: np.ndarray | None


class VisualOdometryTracker:
    """Estimate a continuous camera pose from RGB-D keyframes."""

    def __init__(self, backend: FeatureBackend, config: TrackerConfig) -> None:
        """Create an empty tracker around a feature backend and thresholds."""
        self.backend = backend
        self.config = config
        self._keyframe: _Keyframe | None = None
        self._last_pose: np.ndarray | None = None
        self._consecutive_tracking_failures = 0
        self._odom_from_world_rotation: np.ndarray | None = None

    def reset(self) -> None:
        """Discard tracking state while retaining loaded neural models."""
        self._keyframe = None
        self._last_pose = None
        self._consecutive_tracking_failures = 0
        self._odom_from_world_rotation = None

    @staticmethod
    def _rotation_matrix(rotation: np.ndarray | None) -> np.ndarray | None:
        """Validate an optional external camera orientation."""
        if rotation is None:
            return None
        matrix = np.asarray(rotation, dtype=np.float64)
        if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            raise ValueError("world_from_camera_rotation must be a finite 3x3 matrix")
        u, _, vt = np.linalg.svd(matrix)
        matrix = u @ vt
        if np.linalg.det(matrix) < 0.0:
            u[:, -1] *= -1.0
            matrix = u @ vt
        return matrix

    def _predicted_odom_rotation(
        self, world_from_camera_rotation: np.ndarray | None
    ) -> np.ndarray | None:
        if (
            world_from_camera_rotation is None
            or self._odom_from_world_rotation is None
        ):
            return None
        return self._odom_from_world_rotation @ world_from_camera_rotation

    def _maybe_reseed_keyframe(
        self,
        features: NeuralFeatures,
        points3d: np.ndarray,
        valid_depth: np.ndarray,
        stamp: float,
        world_from_camera_rotation: np.ndarray | None,
    ) -> bool:
        """Replace a stale reference after repeated failures without inventing motion."""
        self._consecutive_tracking_failures += 1
        if (
            self.config.maximum_consecutive_tracking_failures <= 0
            or self._consecutive_tracking_failures
            < self.config.maximum_consecutive_tracking_failures
            or self._last_pose is None
        ):
            return False
        pose = self._last_pose.copy()
        predicted_rotation = self._predicted_odom_rotation(
            world_from_camera_rotation
        )
        if predicted_rotation is not None:
            pose[:3, :3] = predicted_rotation
            self._last_pose = pose.copy()
        self._keyframe = _Keyframe(
            features,
            points3d,
            valid_depth,
            pose,
            stamp,
            None if world_from_camera_rotation is None
            else world_from_camera_rotation.copy(),
        )
        self._consecutive_tracking_failures = 0
        return True

    def _result(
        self,
        started: float,
        features: NeuralFeatures,
        points3d: np.ndarray,
        accepted: bool,
        state: str,
        reason: str,
        pose: np.ndarray | None,
        matches: int = 0,
        depth_matches: int = 0,
        inliers: int = 0,
        ratio: float = 0.0,
        coverage: float = 0.0,
        rmse: float | None = None,
        keyframe_updated: bool = False,
        pose_source: str = "NONE",
        imu_rotation_error: float | None = None,
        depth_consistency_inliers: int = 0,
    ) -> TrackingResult:
        return TrackingResult(
            accepted,
            state,
            reason,
            pose,
            features,
            points3d,
            len(features.keypoints),
            matches,
            depth_matches,
            inliers,
            ratio,
            coverage,
            rmse,
            time.perf_counter() - started,
            keyframe_updated,
            pose_source,
            imu_rotation_error,
            depth_consistency_inliers,
        )

    def process(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        intrinsics: np.ndarray,
        depth_scale: float,
        stamp: float,
        initial_odom_from_camera: np.ndarray | None = None,
        world_from_camera_rotation: np.ndarray | None = None,
    ) -> TrackingResult:
        """Extract, match and geometrically verify one synchronized frame."""
        started = time.perf_counter()
        imu_rotation = self._rotation_matrix(world_from_camera_rotation)
        features = self.backend.extract(rgb)
        points3d, valid_depth = project_depth_points(
            features.keypoints,
            depth,
            intrinsics,
            depth_scale,
            self.config.minimum_depth,
            self.config.maximum_depth,
        )
        if len(features.keypoints) < self.config.minimum_keypoints:
            return self._result(
                started,
                features,
                points3d,
                False,
                "DEGRADED",
                "KEYPOINTS_LOW",
                self._last_pose,
            )

        if self._keyframe is None:
            pose = np.eye(4) if initial_odom_from_camera is None else np.asarray(
                initial_odom_from_camera, dtype=np.float64
            ).copy()
            if imu_rotation is not None:
                self._odom_from_world_rotation = pose[:3, :3] @ imu_rotation.T
            self._keyframe = _Keyframe(
                features, points3d, valid_depth, pose, stamp, imu_rotation
            )
            self._last_pose = pose
            self._consecutive_tracking_failures = 0
            return self._result(
                started,
                features,
                points3d,
                True,
                "TRACKING",
                "INITIALIZED",
                pose,
                keyframe_updated=True,
            )

        reference = self._keyframe
        matches = self.backend.match(reference.features, features)
        reference_indices = np.flatnonzero(matches >= 0)
        current_indices = matches[reference_indices]
        match_count = len(reference_indices)
        if match_count < self.config.minimum_matches:
            reseeded = self._maybe_reseed_keyframe(
                features, points3d, valid_depth, stamp, imu_rotation
            )
            return self._result(
                started, features, points3d, False, "DEGRADED",
                "MATCHES_LOW_KEYFRAME_RESEEDED" if reseeded else "MATCHES_LOW",
                self._last_pose, matches=match_count, keyframe_updated=reseeded,
            )
        depth_mask = reference.valid_depth[reference_indices]
        reference_indices = reference_indices[depth_mask]
        current_indices = current_indices[depth_mask]
        depth_match_count = len(reference_indices)
        if depth_match_count < self.config.minimum_depth_matches:
            reseeded = self._maybe_reseed_keyframe(
                features, points3d, valid_depth, stamp, imu_rotation
            )
            return self._result(
                started, features, points3d, False, "DEGRADED",
                "DEPTH_MATCHES_LOW_KEYFRAME_RESEEDED"
                if reseeded else "DEPTH_MATCHES_LOW",
                self._last_pose, matches=match_count, depth_matches=depth_match_count,
                keyframe_updated=reseeded,
            )
        pose = estimate_relative_pose(
            reference.points3d[reference_indices],
            features.keypoints[current_indices],
            intrinsics,
            self.config.ransac_reprojection_error,
            self.config.ransac_iterations,
            self.config.ransac_confidence,
        )
        if pose is None:
            reseeded = self._maybe_reseed_keyframe(
                features, points3d, valid_depth, stamp, imu_rotation
            )
            return self._result(
                started, features, points3d, False, "LOST",
                "PNP_FAILED_KEYFRAME_RESEEDED" if reseeded else "PNP_FAILED",
                self._last_pose, matches=match_count, depth_matches=depth_match_count,
                keyframe_updated=reseeded,
            )
        pose_source = "PNP"
        imu_rotation_error = 0.0
        depth_consistency_inliers = 0
        if imu_rotation is not None and reference.world_from_camera_rotation is not None:
            imu_current_from_reference = (
                imu_rotation.T @ reference.world_from_camera_rotation
            )
            rotation_delta = np.eye(4, dtype=np.float64)
            rotation_delta[:3, :3] = (
                pose.current_from_reference[:3, :3].T
                @ imu_current_from_reference
            )
            imu_rotation_error = rotation_angle(rotation_delta)
            pnp_match_indices = pose.inlier_indices
            pnp_current_indices = current_indices[pnp_match_indices]
            current_depth_mask = valid_depth[pnp_current_indices]
            consistent_match_positions = np.flatnonzero(current_depth_mask)
            if (
                len(consistent_match_positions)
                >= self.config.minimum_depth_consistency_matches
            ):
                fixed_rotation_pose = estimate_translation_with_rotation(
                    reference.points3d[
                        reference_indices[pnp_match_indices[consistent_match_positions]]
                    ],
                    points3d[pnp_current_indices[consistent_match_positions]],
                    features.keypoints[pnp_current_indices[consistent_match_positions]],
                    intrinsics,
                    imu_current_from_reference,
                    self.config.maximum_depth_consistency_error,
                    self.config.minimum_depth_consistency_matches,
                )
                if (
                    fixed_rotation_pose is not None
                    and fixed_rotation_pose.reprojection_rmse
                    <= self.config.maximum_reprojection_rmse
                ):
                    remapped_inliers = pnp_match_indices[
                        consistent_match_positions[fixed_rotation_pose.inlier_indices]
                    ]
                    pose = type(pose)(
                        fixed_rotation_pose.current_from_reference,
                        remapped_inliers,
                        fixed_rotation_pose.reprojection_rmse,
                    )
                    pose_source = "IMU_DEPTH"
                    depth_consistency_inliers = len(remapped_inliers)

        inlier_count = len(pose.inlier_indices)
        inlier_ratio = inlier_count / float(depth_match_count)
        inlier_pixels = features.keypoints[current_indices[pose.inlier_indices]]
        coverage = image_grid_coverage(
            inlier_pixels, int(features.image_size[0]), int(features.image_size[1])
        )
        rejection = ""
        if (
            pose_source == "PNP"
            and imu_rotation is not None
            and reference.world_from_camera_rotation is not None
            and imu_rotation_error > self.config.maximum_imu_rotation_error
        ):
            rejection = "IMU_ROTATION_MISMATCH"
        elif inlier_count < self.config.minimum_inliers:
            rejection = "INLIERS_LOW"
        elif inlier_ratio < self.config.minimum_inlier_ratio:
            rejection = "INLIER_RATIO_LOW"
        elif coverage < self.config.minimum_grid_coverage:
            rejection = "COVERAGE_LOW"
        elif pose.reprojection_rmse > self.config.maximum_reprojection_rmse:
            rejection = "REPROJECTION_HIGH"

        reference_from_current = invert_transform(pose.current_from_reference)
        odom_from_camera = reference.odom_from_camera @ reference_from_current
        if self._last_pose is not None:
            last_from_current = invert_transform(self._last_pose) @ odom_from_camera
            if np.linalg.norm(last_from_current[:3, 3]) > self.config.maximum_frame_translation:
                rejection = "TRANSLATION_JUMP"
            elif rotation_angle(last_from_current) > self.config.maximum_frame_rotation:
                rejection = "ROTATION_JUMP"
        if rejection:
            reseeded = self._maybe_reseed_keyframe(
                features, points3d, valid_depth, stamp, imu_rotation
            )
            return self._result(
                started, features, points3d, False, "DEGRADED",
                f"{rejection}_KEYFRAME_RESEEDED" if reseeded else rejection,
                self._last_pose, match_count, depth_match_count, inlier_count,
                inlier_ratio, coverage, pose.reprojection_rmse,
                keyframe_updated=reseeded,
                pose_source=pose_source,
                imu_rotation_error=imu_rotation_error,
                depth_consistency_inliers=depth_consistency_inliers,
            )

        reference_from_current_motion = invert_transform(reference.odom_from_camera) @ odom_from_camera
        keyframe_updated = (
            np.linalg.norm(reference_from_current_motion[:3, 3])
            >= self.config.keyframe_min_translation
            or rotation_angle(reference_from_current_motion) >= self.config.keyframe_min_rotation
            or stamp - reference.stamp >= self.config.keyframe_max_age
            or inlier_ratio < self.config.keyframe_min_inlier_ratio
        )
        if keyframe_updated:
            self._keyframe = _Keyframe(
                features,
                points3d,
                valid_depth,
                odom_from_camera.copy(),
                stamp,
                imu_rotation,
            )
        self._last_pose = odom_from_camera
        self._consecutive_tracking_failures = 0
        return self._result(
            started,
            features,
            points3d,
            True,
            "TRACKING",
            "ACCEPTED_IMU_DEPTH" if pose_source == "IMU_DEPTH" else "ACCEPTED",
            odom_from_camera,
            match_count,
            depth_match_count,
            inlier_count,
            inlier_ratio,
            coverage,
            pose.reprojection_rmse,
            keyframe_updated,
            pose_source,
            imu_rotation_error,
            depth_consistency_inliers,
        )
