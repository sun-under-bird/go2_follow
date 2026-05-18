import math
from dataclasses import dataclass
from typing import Iterable, Tuple


@dataclass(frozen=True)
class Transform3D:
    translation: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_rotate(point: Iterable[float], rotation: Iterable[float]) -> Tuple[float, float, float]:
    px, py, pz = point
    qx, qy, qz, qw = rotation

    # q * p
    ix = qw * px + qy * pz - qz * py
    iy = qw * py + qz * px - qx * pz
    iz = qw * pz + qx * py - qy * px
    iw = -qx * px - qy * py - qz * pz

    # result * conj(q)
    rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
    ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
    rz = iz * qw + iw * -qz + ix * -qy - iy * -qx
    return rx, ry, rz


def transform_point(point: Iterable[float], transform: Transform3D) -> Tuple[float, float, float]:
    rx, ry, rz = quaternion_rotate(point, transform.rotation)
    tx, ty, tz = transform.translation
    return rx + tx, ry + ty, rz + tz


def transform_from_ros(transform_stamped) -> Transform3D:
    t = transform_stamped.transform.translation
    r = transform_stamped.transform.rotation
    return Transform3D(
        translation=(float(t.x), float(t.y), float(t.z)),
        rotation=(float(r.x), float(r.y), float(r.z), float(r.w)),
    )
