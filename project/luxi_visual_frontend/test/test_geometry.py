import cv2
import numpy as np

from luxi_visual_frontend.geometry import (
    estimate_relative_pose,
    image_grid_coverage,
    invert_transform,
    project_depth_points,
)


def test_depth_projection_preserves_invalid_rows():
    depth = np.zeros((8, 10), dtype=np.uint16)
    depth[3, 4] = 2000
    points, valid = project_depth_points(
        np.array([[4.0, 3.0], [2.0, 2.0], [20.0, 3.0]]),
        depth,
        np.array([[100.0, 0.0, 5.0], [0.0, 100.0, 4.0], [0.0, 0.0, 1.0]]),
        0.001,
        0.2,
        6.0,
    )
    np.testing.assert_array_equal(valid, [True, False, False])
    np.testing.assert_allclose(points[0], [-0.02, -0.02, 2.0])
    assert np.isnan(points[1:]).all()


def test_relative_pose_recovers_metric_transform_with_outliers():
    random = np.random.default_rng(12)
    reference = random.uniform((-2.0, -1.0, 3.0), (2.0, 1.0, 7.0), (100, 3))
    current_from_reference = np.eye(4)
    rotation_vector = np.array([0.01, -0.04, 0.02])
    current_from_reference[:3, :3], _ = cv2.Rodrigues(rotation_vector)
    current_from_reference[:3, 3] = (0.18, -0.03, 0.04)
    current = (
        reference @ current_from_reference[:3, :3].T
        + current_from_reference[:3, 3]
    )
    intrinsics = np.array(
        [[520.0, 0.0, 320.0], [0.0, 519.0, 240.0], [0.0, 0.0, 1.0]]
    )
    pixels = np.column_stack(
        (
            intrinsics[0, 0] * current[:, 0] / current[:, 2] + intrinsics[0, 2],
            intrinsics[1, 1] * current[:, 1] / current[:, 2] + intrinsics[1, 2],
        )
    )
    pixels += random.normal(0.0, 0.15, pixels.shape)
    pixels[:15] = random.uniform((0.0, 0.0), (640.0, 480.0), (15, 2))
    result = estimate_relative_pose(reference, pixels, intrinsics, 2.0, 1000, 0.999)
    assert result is not None
    assert len(result.inlier_indices) >= 75
    np.testing.assert_allclose(
        result.current_from_reference, current_from_reference, atol=0.015
    )
    assert result.reprojection_rmse < 1.0
    np.testing.assert_allclose(
        invert_transform(invert_transform(current_from_reference)),
        current_from_reference,
    )


def test_grid_coverage_rejects_clustered_features():
    clustered = np.array([[10.0, 10.0], [12.0, 14.0], [20.0, 18.0]])
    distributed = np.array(
        [[80.0 + x * 160.0, 80.0 + y * 160.0] for y in range(3) for x in range(4)]
    )
    assert image_grid_coverage(clustered, 640, 480) == 1.0 / 12.0
    assert image_grid_coverage(distributed, 640, 480) == 1.0
