import cv2
import numpy as np

from luxi_visual_frontend.feature_backend import NeuralFeatures
from luxi_visual_frontend.geometry import RelativePose, invert_transform
import luxi_visual_frontend.tracker as tracker_module
from luxi_visual_frontend.tracker import TrackerConfig, VisualOdometryTracker


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
            use_imu_reseed_rotation=True,
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
    imu_pose = np.eye(4)
    imu_pose[:3, :3], _ = cv2.Rodrigues(
        np.array([0.0, 0.0, np.deg2rad(-20.0)])
    )
    monkeypatch.setattr(
        tracker_module, "estimate_relative_pose", lambda *_args: visual_pose
    )
    monkeypatch.setattr(
        tracker_module,
        "estimate_translation_with_rotation",
        lambda *_args: RelativePose(imu_pose, inliers, 0.1),
    )
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    intrinsics = np.array(
        [[520.0, 0.0, 320.0], [0.0, 520.0, 240.0], [0.0, 0.0, 1.0]]
    )
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    imu_yaw_20deg, _ = cv2.Rodrigues(
        np.array([0.0, 0.0, np.deg2rad(20.0)])
    )

    tracker.process(
        rgb, depth, intrinsics, 0.001, 1.0,
        world_from_camera_rotation=np.eye(3),
    )
    result = tracker.process(
        rgb, depth, intrinsics, 0.001, 1.1,
        world_from_camera_rotation=imu_yaw_20deg,
    )

    assert result.accepted
    assert result.pose_source == "PNP"
    assert result.reason == "ACCEPTED"
    assert np.isclose(np.rad2deg(result.imu_rotation_error), 20.0)
    assert np.isclose(result.imu_gravity_error, 0.0)
    np.testing.assert_allclose(result.odom_from_camera, np.eye(4), atol=1e-6)


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


def test_tracker_default_reseed_never_inherits_unobservable_imu_yaw():
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
            maximum_frame_angular_rate=np.deg2rad(180.0),
        ),
    )
    depth = np.full((480, 640), 2000, dtype=np.uint16)
    intrinsics = np.array(
        [[520.0, 0.0, 320.0], [0.0, 520.0, 240.0], [0.0, 0.0, 1.0]]
    )
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    imu_yaw, _ = cv2.Rodrigues(
        np.array([0.0, 0.0, np.deg2rad(20.0)])
    )

    tracker.process(
        rgb, depth, intrinsics, 0.001, 1.0,
        world_from_camera_rotation=np.eye(3),
    )
    tracker.process(
        rgb, depth, intrinsics, 0.001, 1.1,
        world_from_camera_rotation=imu_yaw,
    )
    reseeded = tracker.process(
        rgb, depth, intrinsics, 0.001, 1.2,
        world_from_camera_rotation=imu_yaw,
    )

    assert not reseeded.accepted and reseeded.keyframe_updated
    np.testing.assert_allclose(reseeded.odom_from_camera, np.eye(4), atol=1e-6)
