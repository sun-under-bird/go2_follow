import numpy as np

from go2_exact_mppi_follow.cloud_filter import (
    CloudFilterConfig,
    filter_cloud_points,
    rotation_matrix_from_quaternion,
    transform_points,
)


FOOTPRINT = [[[0.45, 0.23], [-0.38, 0.23], [-0.38, -0.23], [0.45, -0.23]]]


def test_filter_cloud_removes_robot_body_points():
    """Check that points inside the robot body are not used as obstacles."""

    points = [(0.0, 0.0, 0.4), (1.0, 0.0, 0.4)]
    filtered = filter_cloud_points(points, CloudFilterConfig(max_points=10), FOOTPRINT)
    assert filtered.shape == (1, 2)
    assert np.allclose(filtered[0], [1.0, 0.0])


def test_filter_cloud_applies_roi():
    """Check that points outside the configured ROI are rejected."""

    points = [(4.0, 0.0, 0.4), (1.0, 0.0, 0.4), (1.0, 0.0, 2.0)]
    filtered = filter_cloud_points(points, CloudFilterConfig(max_points=10), FOOTPRINT)
    assert filtered.shape == (1, 2)
    assert np.allclose(filtered[0], [1.0, 0.0])


def test_transform_points_applies_translation():
    """Check rigid point transform translation behavior."""

    rotation = rotation_matrix_from_quaternion(0.0, 0.0, 0.0, 1.0)
    points = transform_points(np.array([[1.0, 2.0, 3.0]], dtype=np.float32), (1.0, 0.0, -1.0), rotation)
    assert np.allclose(points[0], [2.0, 2.0, 2.0])
