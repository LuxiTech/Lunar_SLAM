import math

import numpy as np

from luxi_visual_frontend.node import (
    _matrix_quaternion,
    _quaternion_matrix,
    _tiered_mapping_depth,
)


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
