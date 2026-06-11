"""Pure point-cloud filtering helpers for the Go2 EXACT-MPPI node."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class CloudFilterConfig:
    """Configuration for local point-cloud filtering and downsampling."""

    x_min: float = -0.4
    x_max: float = 3.0
    y_abs: float = 1.8
    z_min: float = 0.05
    z_max: float = 1.0
    voxel_size: float = 0.08
    max_points: int = 300
    corridor_width: float = 0.9


def rotation_matrix_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Build a 3D rotation matrix from a quaternion."""

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return np.eye(3, dtype=np.float32)

    # Normalize first so sensor drivers with slight quaternion drift do not scale points.
    x = qx / norm
    y = qy / norm
    z = qz / norm
    w = qw / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def transform_points(points: np.ndarray, translation: Sequence[float], rotation: np.ndarray) -> np.ndarray:
    """Transform points with a rigid transform into the base frame."""

    if points.size == 0:
        return points.reshape((-1, 3)).astype(np.float32)

    t = np.asarray(translation, dtype=np.float32).reshape((1, 3))
    # Row-vector form keeps the array contiguous for later filtering.
    return (points.astype(np.float32) @ rotation.T) + t


def footprint_bounds(vertices: Sequence[Sequence[Sequence[float]]]) -> Tuple[float, float, float, float]:
    """Return conservative axis-aligned bounds for one or more footprint polygons."""

    flat = [point for polygon in vertices for point in polygon]
    if not flat:
        return -0.38, 0.45, -0.23, 0.23
    xs = [float(point[0]) for point in flat]
    ys = [float(point[1]) for point in flat]
    return min(xs), max(xs), min(ys), max(ys)


def point_inside_bounds_xy(point: np.ndarray, bounds: Tuple[float, float, float, float]) -> bool:
    """Return whether an xy point lies inside conservative robot bounds."""

    x_min, x_max, y_min, y_max = bounds
    return x_min <= float(point[0]) <= x_max and y_min <= float(point[1]) <= y_max


def roi_mask(points: np.ndarray, config: CloudFilterConfig) -> np.ndarray:
    """Return a boolean mask for points inside the local obstacle ROI."""

    if points.size == 0:
        return np.zeros((0,), dtype=bool)

    finite = np.isfinite(points).all(axis=1)
    return (
        finite
        & (points[:, 0] >= config.x_min)
        & (points[:, 0] <= config.x_max)
        & (np.abs(points[:, 1]) <= config.y_abs)
        & (points[:, 2] >= config.z_min)
        & (points[:, 2] <= config.z_max)
    )


def remove_footprint_points(
    points: np.ndarray,
    bounds: Tuple[float, float, float, float],
) -> np.ndarray:
    """Remove points that fall inside the robot footprint bounds."""

    if points.size == 0:
        return points.reshape((-1, 3)).astype(np.float32)

    x_min, x_max, y_min, y_max = bounds
    outside = ~(
        (points[:, 0] >= x_min)
        & (points[:, 0] <= x_max)
        & (points[:, 1] >= y_min)
        & (points[:, 1] <= y_max)
    )
    return points[outside].astype(np.float32)


def voxel_downsample(points_xy: np.ndarray, voxel_size: float) -> np.ndarray:
    """Downsample xy points by keeping one representative per voxel."""

    if points_xy.size == 0:
        return points_xy.reshape((-1, 2)).astype(np.float32)
    if voxel_size <= 1e-6:
        return points_xy.astype(np.float32)

    keys = np.floor(points_xy / float(voxel_size)).astype(np.int32)
    _, unique_idx = np.unique(keys, axis=0, return_index=True)
    unique_idx.sort()
    return points_xy[unique_idx].astype(np.float32)


def obstacle_scores(points_xy: np.ndarray, goal_xy: Optional[Tuple[float, float]], corridor_width: float) -> np.ndarray:
    """Score obstacle points so nearest and goal-corridor points are kept first."""

    if points_xy.size == 0:
        return np.zeros((0,), dtype=np.float32)

    distance_score = np.einsum("ij,ij->i", points_xy, points_xy)
    if goal_xy is None:
        return distance_score.astype(np.float32)

    gx, gy = goal_xy
    goal_len = math.hypot(gx, gy)
    if goal_len <= 1e-6:
        return distance_score.astype(np.float32)

    # Points between robot and target direction are more relevant to follow safety.
    ux = gx / goal_len
    uy = gy / goal_len
    proj = points_xy[:, 0] * ux + points_xy[:, 1] * uy
    lateral = np.abs(points_xy[:, 0] * -uy + points_xy[:, 1] * ux)
    in_corridor = (proj >= -0.1) & (proj <= goal_len + 0.8) & (lateral <= corridor_width * 0.5)
    return (distance_score - in_corridor.astype(np.float32) * 0.5).astype(np.float32)


def select_obstacles(
    points_xy: np.ndarray,
    max_points: int,
    goal_xy: Optional[Tuple[float, float]],
    corridor_width: float,
) -> np.ndarray:
    """Select the most relevant obstacle points for the MPPI obstacle critic."""

    if points_xy.size == 0:
        return points_xy.reshape((-1, 2)).astype(np.float32)

    limit = max(1, int(max_points))
    if points_xy.shape[0] <= limit:
        return points_xy.astype(np.float32)

    scores = obstacle_scores(points_xy, goal_xy, corridor_width)
    idx = np.argpartition(scores, limit - 1)[:limit]
    idx = idx[np.argsort(scores[idx])]
    return points_xy[idx].astype(np.float32)


def filter_cloud_points(
    points: Iterable[Tuple[float, float, float]],
    config: CloudFilterConfig,
    footprint_vertices: Sequence[Sequence[Sequence[float]]],
    goal_xy: Optional[Tuple[float, float]] = None,
) -> np.ndarray:
    """Filter raw base-frame 3D points into local 2D obstacle points."""

    arr = np.asarray(list(points), dtype=np.float32).reshape((-1, 3))
    if arr.size == 0:
        return np.zeros((0, 2), dtype=np.float32)

    bounds = footprint_bounds(footprint_vertices)
    roi_points = arr[roi_mask(arr, config)]
    body_free = remove_footprint_points(roi_points, bounds)
    downsampled = voxel_downsample(body_free[:, :2], config.voxel_size)
    return select_obstacles(downsampled, config.max_points, goal_xy, config.corridor_width)
