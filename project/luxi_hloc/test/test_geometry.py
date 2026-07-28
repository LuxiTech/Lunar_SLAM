import numpy as np

from luxi_hloc.geometry import invert_transform, quaternion_from_rotation, transform_points


def test_transform_inverse_round_trip():
    angle = 0.4
    transform = np.array(
        [
            [np.cos(angle), -np.sin(angle), 0.0, 1.2],
            [np.sin(angle), np.cos(angle), 0.0, -0.3],
            [0.0, 0.0, 1.0, 0.8],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    points = np.array([[0.1, 0.2, 1.0], [-0.4, 0.5, 2.0]])
    recovered = transform_points(invert_transform(transform), transform_points(transform, points))
    np.testing.assert_allclose(recovered, points, atol=1e-10)


def test_quaternion_identity_is_ros_order():
    np.testing.assert_allclose(
        quaternion_from_rotation(np.eye(3)),
        np.array([0.0, 0.0, 0.0, 1.0]),
    )
