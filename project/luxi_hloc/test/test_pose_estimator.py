import numpy as np

from luxi_hloc.geometry import camera_matrix, invert_transform, solve_pnp_ransac


def test_pnp_recovers_metric_camera_pose_with_outliers():
    random = np.random.default_rng(4)
    map_points = random.uniform((-2.0, -1.0, 3.0), (2.0, 1.0, 7.0), size=(80, 3))
    map_from_camera = np.eye(4)
    map_from_camera[:3, 3] = (0.3, -0.2, 0.1)
    camera_from_map = invert_transform(map_from_camera)
    camera_points = (
        map_points @ camera_from_map[:3, :3].T + camera_from_map[:3, 3]
    )
    intrinsics = camera_matrix(520.0, 519.0, 320.0, 240.0)
    image_points = np.column_stack(
        (
            intrinsics[0, 0] * camera_points[:, 0] / camera_points[:, 2] + intrinsics[0, 2],
            intrinsics[1, 1] * camera_points[:, 1] / camera_points[:, 2] + intrinsics[1, 2],
        )
    )
    image_points += random.normal(0.0, 0.2, image_points.shape)
    image_points[:12] = random.uniform((0.0, 0.0), (640.0, 480.0), size=(12, 2))
    result = solve_pnp_ransac(map_points, image_points, intrinsics, 2.0, 1000, 0.999)
    assert result is not None
    assert len(result.inlier_indices) >= 60
    np.testing.assert_allclose(
        invert_transform(result.camera_from_map)[:3, 3],
        map_from_camera[:3, 3],
        atol=0.02,
    )
    assert result.reprojection_rmse < 1.0
