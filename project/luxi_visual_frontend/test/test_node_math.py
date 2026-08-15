import math

import numpy as np

from luxi_visual_frontend.node import (
    _apply_gravity_ground_plane_prior,
    _matrix_quaternion,
    _quaternion_matrix,
    _tiered_mapping_depth,
)


def test_gravity_ground_prior_repairs_below_floor_and_preserves_obstacles():
    depth = np.zeros((6, 5), dtype=np.uint16)
    depth[5, 1] = 2000
    depth[5, 2] = 500
    intrinsics = np.array(
        [[5.0, 0.0, 2.0], [0.0, 5.0, 1.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    # Optical x/right -> world -y, optical y/down -> world -z and optical
    # z/forward -> world +x.
    world_from_camera = np.array(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    )

    corrected, mask = _apply_gravity_ground_plane_prior(
        depth, 0.001, intrinsics, world_from_camera, 0.5, 0.1, 0.4, 10.0
    )

    assert corrected[5, 1] == 625
    assert mask[5, 1]
    assert corrected[5, 2] == 500
    assert not mask[5, 2]
    assert corrected[5, 3] == 0


def test_gravity_ground_prior_follows_camera_pitch_above_image_center():
    depth = np.zeros((5, 5), dtype=np.uint16)
    depth[2, 2] = 4000
    intrinsics = np.array(
        [[5.0, 0.0, 2.0], [0.0, 5.0, 2.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    nominal = np.array(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    )
    angle = math.radians(-20.0)
    pitch = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(angle), -math.sin(angle)],
            [0.0, math.sin(angle), math.cos(angle)],
        ]
    )

    corrected, mask = _apply_gravity_ground_plane_prior(
        depth, 0.001, intrinsics, nominal @ pitch, 0.5, 0.1, 0.4, 10.0
    )

    assert mask[2, 2]
    assert abs(int(corrected[2, 2]) - 1462) <= 1


def test_gravity_ground_prior_snaps_near_floor_but_keeps_low_obstacle():
    depth = np.zeros((6, 5), dtype=np.uint16)
    # At row 5 the down component is 0.8. These returns reconstruct 5 cm and
    # 15 cm above a 0.5 m floor respectively.
    depth[5, 1] = 562
    depth[5, 2] = 438
    intrinsics = np.array(
        [[5.0, 0.0, 2.0], [0.0, 5.0, 1.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    world_from_camera = np.array(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    )

    corrected, mask = _apply_gravity_ground_plane_prior(
        depth, 0.001, intrinsics, world_from_camera, 0.5, 0.1, 0.4, 10.0, 0.08
    )

    assert corrected[5, 1] == 625
    assert mask[5, 1]
    assert corrected[5, 2] == 438
    assert not mask[5, 2]


def test_gravity_ground_prior_does_not_stretch_large_error_onto_floor():
    depth = np.zeros((6, 5), dtype=np.uint16)
    depth[5, 1] = 2000  # 1.1 m below the expected 0.5 m plane.
    intrinsics = np.array(
        [[5.0, 0.0, 2.0], [0.0, 5.0, 1.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    world_from_camera = np.array(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    )

    corrected, mask = _apply_gravity_ground_plane_prior(
        depth, 0.001, intrinsics, world_from_camera,
        0.5, 0.05, 0.4, 10.0, 0.0, 0.12
    )

    assert corrected[5, 1] == 2000
    assert not mask[5, 1]


def test_gravity_ground_prior_uses_surface_normal_to_select_floor_not_wall():
    height = width = 9
    intrinsics = np.array(
        [[10.0, 0.0, 4.0], [0.0, 10.0, 2.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    world_from_camera = np.array(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    )
    rows = (np.arange(height, dtype=np.float32)[:, None] - 2.0) / 10.0
    # A plane parallel to the floor but reconstructed 50% too low.
    floor = np.zeros((height, width), dtype=np.float32)
    floor[3:] = np.broadcast_to(0.75 / rows[3:], (height - 3, width))
    corrected_floor, floor_mask = _apply_gravity_ground_plane_prior(
        floor, 1.0, intrinsics, world_from_camera,
        0.5, 0.05, 0.4, 10.0, 0.0, 1.0, 0.75, 0.4
    )

    assert floor_mask[6, 4]
    assert abs(float(corrected_floor[6, 4]) - 1.25) < 1e-5

    # A fronto-parallel wall can lie below the expected ray-plane
    # intersection numerically, but its local normal is horizontal.
    wall = np.full((height, width), 3.0, dtype=np.float32)
    corrected_wall, wall_mask = _apply_gravity_ground_plane_prior(
        wall, 1.0, intrinsics, world_from_camera,
        0.5, 0.05, 0.4, 10.0, 0.0, 1.0, 0.75, 0.4
    )

    assert not np.any(wall_mask)
    np.testing.assert_array_equal(corrected_wall, wall)

    rejected_wall, rejected_mask = _apply_gravity_ground_plane_prior(
        wall, 1.0, intrinsics, world_from_camera,
        0.5, 0.05, 0.4, 10.0, 0.0, 1.0, 0.75, 0.4, True
    )
    assert not np.any(rejected_mask)
    assert np.count_nonzero(rejected_wall[4:]) == 0


def test_gravity_ground_prior_repairs_every_ray_behind_support_plane():
    depth = np.zeros((6, 5), dtype=np.uint16)
    depth[5, 1] = 2000  # Invalid: the ray continues far behind the floor.
    depth[5, 2] = 500   # Valid: a nearer obstacle remains above the floor.
    intrinsics = np.array(
        [[5.0, 0.0, 2.0], [0.0, 5.0, 1.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    world_from_camera = np.array(
        [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
        dtype=np.float64,
    )

    corrected, mask = _apply_gravity_ground_plane_prior(
        depth,
        0.001,
        intrinsics,
        world_from_camera,
        0.5,
        0.05,
        0.4,
        10.0,
        0.0,
        0.1,
        0.99,
        0.4,
        True,
        True,
    )

    assert mask[5, 1]
    assert corrected[5, 1] == 625
    assert not mask[5, 2]
    assert corrected[5, 2] == 500


def test_quaternion_matrix_is_a_proper_rotation_for_general_attitude():
    """A valid AHRS quaternion must never introduce reflection or scaling."""
    axis = np.array([0.37, -0.51, 0.78], dtype=np.float64)
    axis /= np.linalg.norm(axis)
    angle = math.radians(73.0)
    xyz = axis * math.sin(angle * 0.5)
    rotation = _quaternion_matrix(*xyz, math.cos(angle * 0.5))

    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
    np.testing.assert_allclose(np.linalg.det(rotation), 1.0, atol=1e-12)


def test_quaternion_rotation_round_trip_keeps_direction_and_handedness():
    quaternion = (0.31, -0.27, 0.19, 0.892)
    rotation = _quaternion_matrix(*quaternion)
    recovered = np.array(_matrix_quaternion(rotation))
    normalized = np.array(quaternion) / np.linalg.norm(quaternion)

    assert abs(float(np.dot(recovered, normalized))) > 1.0 - 1e-12
    assert np.linalg.det(rotation) > 0.0


def test_tiered_mapping_depth_keeps_dense_near_and_sparse_far():
    depth = np.array(
        [
            [200, 1000, 1000, 1000, 5000, 5000],
            [1000, 1000, 1000, 1000, 5000, 5000],
            [1000, 1000, 11000, 1000, 5000, 5000],
            [5000, 5000, 5000, 5000, 5000, 5000],
            [5000, 5000, 5000, 5000, 5000, 5000],
            [5000, 5000, 5000, 5000, 5000, 5000],
        ],
        dtype=np.uint16,
    )

    layered = _tiered_mapping_depth(depth, 0.001, 0.4, 4.0, 10.0, 3)

    assert layered[0, 0] == 0
    assert layered[0, 1] == 1000
    assert layered[2, 2] == 0
    assert layered[0, 3] == 1000
    assert layered[3, 0] == 5000
    assert layered[3, 3] == 5000
    assert layered[4, 4] == 0


def test_tiered_mapping_depth_disabled_returns_original_array():
    depth = np.ones((3, 4), dtype=np.float32)
    assert _tiered_mapping_depth(depth, 1.0, 0.0, 0.0, 0.0, 1) is depth


def test_tiered_mapping_depth_keeps_dense_maximum_boundary():
    depth = np.full((2, 2), 4000, dtype=np.uint16)

    layered = _tiered_mapping_depth(depth, 0.001, 0.4, 4.0, 10.0, 6)

    np.testing.assert_array_equal(layered, depth)


def test_tiered_mapping_depth_uses_finer_medium_than_far_lattice():
    depth = np.full((9, 9), 7000, dtype=np.uint16)
    depth[:, :4] = 5000

    layered = _tiered_mapping_depth(
        depth, 0.001, 0.4, 4.0, 10.0, 8, 6.0, 4
    )

    # 5 m samples use the 4 px lattice; 7 m samples use the 8 px lattice.
    assert layered[4, 0] == 5000
    assert layered[4, 8] == 0
    assert layered[8, 8] == 7000
    assert layered[4, 4] == 0
