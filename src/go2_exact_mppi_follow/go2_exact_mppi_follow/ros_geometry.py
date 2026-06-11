"""Small ROS geometry helpers kept independent from controller logic."""

from __future__ import annotations

import math
from typing import Tuple


def quaternion_to_yaw(q) -> float:
    """Extract planar yaw from a ROS quaternion-like object."""

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float):
    """Create a geometry_msgs Quaternion for a planar yaw angle."""

    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def transform_point_xy(point_xy: Tuple[float, float], transform) -> Tuple[float, float]:
    """Transform a planar point with a ROS TransformStamped."""

    yaw = quaternion_to_yaw(transform.transform.rotation)
    c = math.cos(yaw)
    s = math.sin(yaw)
    x, y = point_xy
    tx = transform.transform.translation.x
    ty = transform.transform.translation.y
    return c * x - s * y + tx, s * x + c * y + ty
