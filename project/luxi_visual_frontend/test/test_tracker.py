import cv2
import numpy as np

from luxi_visual_frontend.feature_backend import NeuralFeatures
from luxi_visual_frontend.geometry import invert_transform
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
