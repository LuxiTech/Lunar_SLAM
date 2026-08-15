import cv2
import numpy as np

from luxi_visual_frontend.feature_backend import NeuralFeatures
from luxi_visual_frontend.geometry import RelativePose, invert_transform
import luxi_visual_frontend.tracker as tracker_module
from luxi_visual_frontend.tracker import (
    TrackerConfig,
    VisualOdometryTracker,
    constrain_camera_pose_vertical_translation,
)


class SequenceBackend:
    def __init__(self, features):
        self.features = iter(features)

    def extract(self, _rgb):
        return next(self.features)

    @staticmethod
    def match(first, second):
        assert len(first.keypoints) == len(second.keypoints)
        return np.arange(len(first.keypoints), dtype=np.int64)


class MatchSequenceBackend(SequenceBackend):
    def __init__(self, features, matches):
        super().__init__(features)
        self.matches = iter(matches)

    def match(self, _first, _second):
        return next(self.matches)


def test_vertical_constraint_locks_base_height_but_preserves_attitude():
    base_from_camera = np.eye(4)
    base_from_camera[:3, 3] = (0.20, 0.045, 0.20)
    base_from_camera[:3, :3], _ = cv2.Rodrigues(
        np.array([0.12, -0.08, 0.03], dtype=np.float64)
    )
    odom_from_base = np.eye(4)
    odom_from_base[:3, 3] = (1.3, -0.4, -0.29)
    odom_from_base[:3, :3], _ = cv2.Rodrigues(
        np.array([0.04, -0.06, 0.7], dtype=np.float64)
    )

    constrained_camera = constrain_camera_pose_vertical_translation(
        odom_from_base @ base_from_camera, base_from_camera
    )
    constrained_base = constrained_camera @ invert_transform(base_from_camera)

    np.testing.assert_allclose(constrained_base[:2, 3], (1.3, -0.4), atol=1e-12)
    assert abs(constrained_base[2, 3]) < 1e-12
    np.testing.assert_allclose(
        constrained_base[:3, :3], odom_from_base[:3, :3], atol=1e-12
    )


def _features(keypoints):
    count = len(keypoints)
    descriptors = np.zeros((count, 256), dtype=np.float32)
    descriptors[:, 0] = np.arange(count)
    return NeuralFeatures(
        np.asarray(keypoints, dtype=np.float32),
        descriptors,
        np.ones(count, dtype=np.float32),
        np.array([640.0, 480.0], dtype=np.float32),
    )


def test_tracker_estimates_camera_pose_and_keeps_metric_scale():
    intrinsics = np.array(
        [[520.0, 0.0, 320.0], [0.0, 519.0, 240.0], [0.0, 0.0, 1.0]]
    )
    reference_pixels = np.array(
        [[80.0 + x * 48.0, 70.0 + y * 45.0] for y in range(8) for x in range(10)]
    )
    depth = np.zeros((480, 640), dtype=np.uint16)
    reference_points = []
    for index, (u, v) in enumerate(reference_pixels):
        z = 3.0 + index * 0.025
        depth[round(v), round(u)] = round(z * 1000.0)
        reference_points.append(
            [(u - 320.0) * z / 520.0, (v - 240.0) * z / 519.0, z]
        )
    reference_points = np.asarray(reference_points)
    current_from_reference = np.eye(4)
    current_from_reference[:3, :3], _ = cv2.Rodrigues(np.array([0.01, -0.03, 0.02]))
    current_from_reference[:3, 3] = (0.16, -0.02, 0.03)
    current_points = (
        reference_points @ current_from_reference[:3, :3].T
        + current_from_reference[:3, 3]
    )
    current_pixels = np.column_stack(
        (
            520.0 * current_points[:, 0] / current_points[:, 2] + 320.0,
            519.0 * current_points[:, 1] / current_points[:, 2] + 240.0,
        )
    )
    tracker = VisualOdometryTracker(
        SequenceBackend([_features(reference_pixels), _features(current_pixels)]),
        TrackerConfig(
            minimum_keypoints=40,
            minimum_matches=30,
            minimum_depth_matches=30,
            minimum_inliers=25,
            minimum_grid_coverage=0.2,
            keyframe_min_translation=1.0,
            keyframe_max_age=10.0,
        ),
    )
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    initial = np.eye(4)
    first = tracker.process(rgb, depth, intrinsics, 0.001, 1.0, initial)
    second = tracker.process(rgb, depth, intrinsics, 0.001, 1.1, initial)
    assert first.accepted and first.reason == "INITIALIZED"
    assert second.accepted and second.inlier_count >= 70
    expected = invert_transform(current_from_reference)
    np.testing.assert_allclose(second.odom_from_camera, expected, atol=0.02)


def test_tracker_does_not_initialize_with_too_few_features():
    features = _features(np.array([[10.0, 10.0], [20.0, 20.0]]))
    tracker = VisualOdometryTracker(
        SequenceBackend([features]), TrackerConfig(minimum_keypoints=3)
    )
    result = tracker.process(
        np.zeros((40, 40, 3), dtype=np.uint8),
        np.ones((40, 40), dtype=np.uint16) * 1000,
        np.array([[20.0, 0.0, 20.0], [0.0, 20.0, 20.0], [0.0, 0.0, 1.0]]),
        0.001,
        1.0,
    )
    assert not result.accepted
    assert result.reason == "KEYPOINTS_LOW"


def test_tracker_reseeds_keyframe_after_consecutive_match_failures():
    pixels = np.array(
        [[80.0 + x * 70.0, 80.0 + y * 70.0] for y in range(3) for x in range(4)]
    )
    features = [_features(pixels) for _ in range(4)]
    no_matches = np.full(len(pixels), -1, dtype=np.int64)
    all_matches = np.arange(len(pixels), dtype=np.int64)
    tracker = VisualOdometryTracker(
        MatchSequenceBackend(features, [no_matches, no_matches, all_matches]),
        TrackerConfig(
            minimum_keypoints=8,
            minimum_matches=8,
            minimum_depth_matches=8,
            minimum_inliers=6,
            minimum_grid_coverage=0.0,
            maximum_consecutive_tracking_failures=2,
        ),
    )
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    intrinsics = np.array(
        [[520.0, 0.0, 320.0], [0.0, 520.0, 240.0], [0.0, 0.0, 1.0]]
    )
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)

    initialized = tracker.process(rgb, depth, intrinsics, 0.001, 1.0)
    first_failure = tracker.process(rgb, depth, intrinsics, 0.001, 1.1)
    reseeded = tracker.process(rgb, depth, intrinsics, 0.001, 1.2)
    recovered = tracker.process(rgb, depth, intrinsics, 0.001, 1.3)

    assert initialized.accepted
    assert not first_failure.accepted and not first_failure.keyframe_updated
    assert not reseeded.accepted and reseeded.keyframe_updated
    assert reseeded.reason == "MATCHES_LOW_KEYFRAME_RESEEDED"
    assert recovered.accepted


def test_tracker_reseed_propagates_from_last_accepted_visual_pose():
    pixels = np.array(
        [[80.0 + x * 70.0, 80.0 + y * 70.0] for y in range(3) for x in range(4)]
    )
    features = [_features(pixels) for _ in range(4)]
    no_matches = np.full(len(pixels), -1, dtype=np.int64)
    all_matches = np.arange(len(pixels), dtype=np.int64)
    tracker = VisualOdometryTracker(
        MatchSequenceBackend(features, [no_matches, no_matches, all_matches]),
        TrackerConfig(
            minimum_keypoints=8,
            minimum_matches=8,
            minimum_depth_matches=8,
            minimum_inliers=6,
            minimum_grid_coverage=0.0,
            minimum_depth_consistency_matches=6,
            maximum_consecutive_tracking_failures=2,
            maximum_frame_angular_rate=np.deg2rad(180.0),
        ),
    )
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    intrinsics = np.array(
        [[520.0, 0.0, 320.0], [0.0, 520.0, 240.0], [0.0, 0.0, 1.0]]
    )
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    rotation_20deg, _ = cv2.Rodrigues(np.array([0.0, 0.0, np.deg2rad(20.0)]))

    initialized = tracker.process(
        rgb, depth, intrinsics, 0.001, 1.0,
        world_from_camera_rotation=np.eye(3),
    )
    visual_correction, _ = cv2.Rodrigues(
        np.array([0.0, 0.0, np.deg2rad(15.0)])
    )
    corrected_pose = initialized.odom_from_camera.copy()
    corrected_pose[:3, :3] = visual_correction
    tracker._last_pose = corrected_pose.copy()
    tracker._last_accepted_pose = corrected_pose.copy()
    tracker._last_accepted_world_from_camera_rotation = np.eye(3)
    first_failure = tracker.process(
        rgb, depth, intrinsics, 0.001, 1.1,
        world_from_camera_rotation=rotation_20deg,
    )
    reseeded = tracker.process(
        rgb, depth, intrinsics, 0.001, 1.2,
        world_from_camera_rotation=rotation_20deg,
    )
    recovered = tracker.process(
        rgb, depth, intrinsics, 0.001, 1.3,
        world_from_camera_rotation=rotation_20deg,
    )

    assert initialized.accepted
    assert not first_failure.accepted
    assert not reseeded.accepted and reseeded.keyframe_updated
    expected_rotation = visual_correction @ rotation_20deg
    np.testing.assert_allclose(
        reseeded.odom_from_camera[:3, :3], expected_rotation, atol=1e-6
    )
    assert recovered.accepted
    np.testing.assert_allclose(
        recovered.odom_from_camera[:3, :3], expected_rotation, atol=1e-6
    )
    np.testing.assert_allclose(recovered.odom_from_camera[:3, 3], 0.0, atol=1e-6)


def test_tracker_keeps_visual_yaw_when_full_imu_rotation_disagrees(monkeypatch):
    pixels = np.array(
        [[80.0 + x * 70.0, 80.0 + y * 70.0] for y in range(3) for x in range(4)]
    )
    features = [_features(pixels), _features(pixels)]
    tracker = VisualOdometryTracker(
        SequenceBackend(features),
        TrackerConfig(
            minimum_keypoints=8,
            minimum_matches=8,
            minimum_depth_matches=8,
            minimum_inliers=6,
            minimum_grid_coverage=0.0,
            minimum_depth_consistency_matches=6,
            maximum_imu_rotation_error=np.deg2rad(12.0),
            maximum_imu_gravity_error=np.deg2rad(10.0),
        ),
    )
    inliers = np.arange(len(pixels), dtype=np.int64)
    visual_pose = RelativePose(np.eye(4), inliers, 0.1)
    monkeypatch.setattr(
        tracker_module, "estimate_relative_pose", lambda *_args: visual_pose
    )

    def fixed_rotation_pose(*args):
        transform = np.eye(4)
        transform[:3, :3] = args[4]
        return RelativePose(transform, inliers, 0.1)

    monkeypatch.setattr(
        tracker_module,
        "estimate_translation_with_rotation",
        fixed_rotation_pose,
    )
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    intrinsics = np.array(
        [[520.0, 0.0, 320.0], [0.0, 520.0, 240.0], [0.0, 0.0, 1.0]]
    )
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    imu_yaw_4deg, _ = cv2.Rodrigues(
        np.array([0.0, 0.0, np.deg2rad(4.0)])
    )

    tracker.process(
        rgb, depth, intrinsics, 0.001, 1.0,
        world_from_camera_rotation=np.eye(3),
    )
    result = tracker.process(
        rgb, depth, intrinsics, 0.001, 1.1,
        world_from_camera_rotation=imu_yaw_4deg,
    )

    assert result.accepted
    assert result.pose_source == "IMU_PNP"
    assert result.reason == "ACCEPTED_IMU_PNP"
    assert np.isclose(np.rad2deg(result.imu_rotation_error), 4.0)
    assert np.isclose(result.imu_gravity_error, 0.0)
    np.testing.assert_allclose(result.odom_from_camera, np.eye(4), atol=1e-6)


def test_tracker_holds_pose_for_stationary_images_with_imu(monkeypatch):
    pixels = np.array(
        [[80.0 + x * 70.0, 80.0 + y * 70.0] for y in range(3) for x in range(4)]
    )
    tracker = VisualOdometryTracker(
        SequenceBackend([_features(pixels), _features(pixels)]),
        TrackerConfig(
            minimum_keypoints=8,
            minimum_matches=8,
            minimum_depth_matches=8,
            minimum_inliers=6,
            minimum_grid_coverage=0.0,
            minimum_depth_consistency_matches=6,
        ),
    )
    inliers = np.arange(len(pixels), dtype=np.int64)
    noisy_translation = np.eye(4)
    noisy_translation[:3, 3] = (0.04, -0.02, 0.03)
    monkeypatch.setattr(
        tracker_module,
        "estimate_relative_pose",
        lambda *_args: RelativePose(noisy_translation, inliers, 0.2),
    )
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    intrinsics = np.array(
        [[520.0, 0.0, 320.0], [0.0, 520.0, 240.0], [0.0, 0.0, 1.0]]
    )
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)

    tracker.process(
        rgb, depth, intrinsics, 0.001, 1.0,
        world_from_camera_rotation=np.eye(3),
    )
    result = tracker.process(
        rgb, depth, intrinsics, 0.001, 1.1,
        world_from_camera_rotation=np.eye(3),
    )

    assert result.accepted
    assert result.pose_source == "STATIONARY"
    assert result.reason == "ACCEPTED_STATIONARY"
    np.testing.assert_allclose(result.odom_from_camera, np.eye(4), atol=1e-9)


def test_tracker_uses_zero_motion_command_to_reject_crestereo_drift(monkeypatch):
    pixels = np.array(
        [[80.0 + x * 70.0, 80.0 + y * 70.0] for y in range(3) for x in range(4)]
    )
    shifted_pixels = pixels + np.array([1.0, 0.0], dtype=np.float32)
    config = TrackerConfig(
        minimum_keypoints=8,
        minimum_matches=8,
        minimum_depth_matches=8,
        minimum_inliers=6,
        minimum_grid_coverage=0.0,
        minimum_depth_consistency_matches=6,
        stationary_maximum_median_pixel_motion=0.75,
        stationary_hint_maximum_median_pixel_motion=2.0,
        stationary_hint_maximum_translation=0.05,
    )
    inliers = np.arange(len(pixels), dtype=np.int64)
    noisy_translation = np.eye(4)
    noisy_translation[0, 3] = 0.04
    monkeypatch.setattr(
        tracker_module,
        "estimate_relative_pose",
        lambda *_args: RelativePose(noisy_translation, inliers, 0.2),
    )
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    intrinsics = np.array(
        [[520.0, 0.0, 320.0], [0.0, 520.0, 240.0], [0.0, 0.0, 1.0]]
    )
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)

    hinted_tracker = VisualOdometryTracker(
        SequenceBackend([_features(pixels), _features(shifted_pixels)]), config
    )
    hinted_tracker.process(
        rgb, depth, intrinsics, 0.001, 1.0,
        world_from_camera_rotation=np.eye(3),
    )
    held = hinted_tracker.process(
        rgb, depth, intrinsics, 0.001, 1.1,
        world_from_camera_rotation=np.eye(3),
        stationary_hint=True,
    )

    moving_tracker = VisualOdometryTracker(
        SequenceBackend([_features(pixels), _features(shifted_pixels)]), config
    )
    moving_tracker.process(
        rgb, depth, intrinsics, 0.001, 1.0,
        world_from_camera_rotation=np.eye(3),
    )
    moving = moving_tracker.process(
        rgb, depth, intrinsics, 0.001, 1.1,
        world_from_camera_rotation=np.eye(3),
        stationary_hint=False,
    )

    assert held.accepted and held.reason == "ACCEPTED_STATIONARY_HINT"
    assert held.pose_source == "STATIONARY_HINT"
    assert held.median_pixel_motion == 1.0
    np.testing.assert_allclose(held.odom_from_camera, np.eye(4), atol=1e-9)
    assert moving.accepted and moving.reason == "ACCEPTED_IMU_PNP"
    assert moving.pose_source == "IMU_PNP"
    assert abs(float(moving.odom_from_camera[0, 3])) > 0.03


def test_gravity_alignment_corrects_tilt_without_using_imu_yaw():
    visual_yaw, _ = cv2.Rodrigues(
        np.array([0.0, 0.0, np.deg2rad(7.0)])
    )
    imu_tilt, _ = cv2.Rodrigues(
        np.array([np.deg2rad(5.0), np.deg2rad(-3.0), np.deg2rad(20.0)])
    )
    aligned = VisualOdometryTracker._align_visual_rotation_to_gravity(
        visual_yaw, np.eye(3), imu_tilt
    )
    target_up = imu_tilt.T @ np.array([0.0, 0.0, 1.0])

    np.testing.assert_allclose(
        aligned @ np.array([0.0, 0.0, 1.0]), target_up, atol=1e-7
    )
    np.testing.assert_allclose(aligned.T @ aligned, np.eye(3), atol=1e-7)
    assert np.isclose(np.linalg.det(aligned), 1.0)


def test_tracker_rejects_jump_hidden_by_reseeded_internal_pose(monkeypatch):
    pixels = np.array(
        [[80.0 + x * 70.0, 80.0 + y * 70.0] for y in range(3) for x in range(4)]
    )
    tracker = VisualOdometryTracker(
        SequenceBackend([_features(pixels), _features(pixels)]),
        TrackerConfig(
            minimum_keypoints=8,
            minimum_matches=8,
            minimum_depth_matches=8,
            minimum_inliers=6,
            minimum_grid_coverage=0.0,
            minimum_depth_consistency_matches=20,
            maximum_frame_rotation=np.deg2rad(15.0),
        ),
    )
    inliers = np.arange(len(pixels), dtype=np.int64)
    monkeypatch.setattr(
        tracker_module,
        "estimate_relative_pose",
        lambda *_args: RelativePose(np.eye(4), inliers, 0.1),
    )
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    intrinsics = np.array(
        [[520.0, 0.0, 320.0], [0.0, 520.0, 240.0], [0.0, 0.0, 1.0]]
    )
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    initialized = tracker.process(rgb, depth, intrinsics, 0.001, 1.0)
    bad_pose = initialized.odom_from_camera.copy()
    bad_pose[:3, :3], _ = cv2.Rodrigues(
        np.array([0.0, 0.0, np.deg2rad(60.0)])
    )
    tracker._keyframe.odom_from_camera = bad_pose.copy()
    tracker._last_pose = bad_pose.copy()

    result = tracker.process(rgb, depth, intrinsics, 0.001, 1.1)

    assert not result.accepted
    assert result.reason == "ROTATION_JUMP"


def test_tracker_does_not_reseed_from_impossible_imu_angular_rate():
    pixels = np.array(
        [[80.0 + x * 70.0, 80.0 + y * 70.0] for y in range(3) for x in range(4)]
    )
    features = [_features(pixels) for _ in range(3)]
    no_matches = np.full(len(pixels), -1, dtype=np.int64)
    tracker = VisualOdometryTracker(
        MatchSequenceBackend(features, [no_matches, no_matches]),
        TrackerConfig(
            minimum_keypoints=8,
            minimum_matches=8,
            minimum_depth_matches=8,
            minimum_inliers=6,
            minimum_grid_coverage=0.0,
            maximum_consecutive_tracking_failures=2,
            maximum_frame_angular_rate=np.deg2rad(90.0),
        ),
    )
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    intrinsics = np.array(
        [[520.0, 0.0, 320.0], [0.0, 520.0, 240.0], [0.0, 0.0, 1.0]]
    )
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    impossible_rotation, _ = cv2.Rodrigues(
        np.array([0.0, 0.0, np.deg2rad(60.0)])
    )

    tracker.process(
        rgb, depth, intrinsics, 0.001, 1.0,
        world_from_camera_rotation=np.eye(3),
    )
    tracker.process(
        rgb, depth, intrinsics, 0.001, 1.1,
        world_from_camera_rotation=impossible_rotation,
    )
    reseeded = tracker.process(
        rgb, depth, intrinsics, 0.001, 1.2,
        world_from_camera_rotation=impossible_rotation,
    )

    assert not reseeded.accepted and reseeded.keyframe_updated
    np.testing.assert_allclose(reseeded.odom_from_camera, np.eye(4), atol=1e-6)


def test_tracker_refines_pnp_translation_with_current_depth():
    pixels = np.array(
        [[80.0 + x * 70.0, 80.0 + y * 70.0] for y in range(3) for x in range(4)]
    )
    features = [_features(pixels), _features(pixels)]
    tracker = VisualOdometryTracker(
        SequenceBackend(features),
        TrackerConfig(
            minimum_keypoints=8,
            minimum_matches=8,
            minimum_depth_matches=8,
            minimum_inliers=6,
            minimum_grid_coverage=0.0,
            use_depth_translation_refinement=True,
            minimum_depth_consistency_matches=6,
        ),
    )
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    intrinsics = np.array(
        [[520.0, 0.0, 320.0], [0.0, 520.0, 240.0], [0.0, 0.0, 1.0]]
    )
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)

    initialized = tracker.process(rgb, depth, intrinsics, 0.001, 1.0)
    refined = tracker.process(rgb, depth, intrinsics, 0.001, 1.1)

    assert initialized.accepted
    assert refined.accepted
    assert refined.pose_source == "PNP_DEPTH"
    assert refined.depth_consistency_inliers >= 6
    np.testing.assert_allclose(refined.odom_from_camera, np.eye(4), atol=1e-6)


def test_tracker_keeps_pnp_when_depth_refinement_has_too_few_inliers(monkeypatch):
    """A small 3-D subset must not downgrade an accepted visual pose."""
    pixels = np.array(
        [[80.0 + x * 70.0, 80.0 + y * 70.0] for y in range(4) for x in range(8)]
    )
    tracker = VisualOdometryTracker(
        SequenceBackend([_features(pixels), _features(pixels)]),
        TrackerConfig(
            minimum_keypoints=20,
            minimum_matches=20,
            minimum_depth_matches=20,
            minimum_inliers=25,
            minimum_grid_coverage=0.0,
            use_depth_translation_refinement=True,
            minimum_depth_consistency_matches=20,
        ),
    )
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    intrinsics = np.array(
        [[520.0, 0.0, 320.0], [0.0, 520.0, 240.0], [0.0, 0.0, 1.0]]
    )
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    tracker.process(rgb, depth, intrinsics, 0.001, 1.0)

    import luxi_visual_frontend.tracker as tracker_module

    original_refinement = tracker_module.estimate_translation_with_rotation

    def truncated_refinement(*args, **kwargs):
        result = original_refinement(*args, **kwargs)
        assert result is not None
        return type(result)(
            result.current_from_reference,
            result.inlier_indices[:20],
            result.reprojection_rmse,
        )

    monkeypatch.setattr(
        tracker_module, "estimate_translation_with_rotation", truncated_refinement
    )
    result = tracker.process(rgb, depth, intrinsics, 0.001, 1.1)

    assert result.accepted
    assert result.pose_source == "PNP"
    assert result.inlier_count >= 25
