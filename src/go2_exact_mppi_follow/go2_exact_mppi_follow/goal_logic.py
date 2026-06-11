"""Pure target parsing and follow-goal helpers for the Go2 EXACT-MPPI nodes."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple


@dataclass(frozen=True)
class UwbParseConfig:
    """Configuration for parsing ONE1000/UWB target measurements."""

    prefer_xy: bool = True
    angle_in_degrees: bool = False
    invert_y: bool = False
    angle_offset_rad: float = 0.0
    anchor_x_offset: float = 0.0
    anchor_y_offset: float = 0.0
    min_target_distance: float = 0.15
    max_target_distance: float = 8.0


@dataclass(frozen=True)
class FollowGoalConfig:
    """Configuration for converting a target position into a local follow goal."""

    follow_distance: float = 0.9
    forward_goal_limit: float = 2.0
    reverse_goal_limit: float = 0.4
    goal_deadband: float = 0.03


def is_finite(value: float) -> bool:
    """Return whether a value is finite enough for control math."""

    return math.isfinite(float(value))


def clamp(value: float, low: float, high: float) -> float:
    """Clamp a scalar into a closed interval."""

    return max(low, min(high, value))


def validate_target(point: Tuple[float, float], config: UwbParseConfig) -> Optional[Tuple[float, float]]:
    """Validate target coordinates and distance gates."""

    x, y = point
    if not is_finite(x) or not is_finite(y):
        return None

    distance = math.hypot(x, y)
    if distance < config.min_target_distance or distance > config.max_target_distance:
        return None
    return x, y


def parse_uwb_target(
    x: float,
    y: float,
    range_m: float,
    angle: float,
    config: UwbParseConfig,
) -> Optional[Tuple[float, float]]:
    """Parse a UWB message into target coordinates in the sensor frame."""

    xy_valid = is_finite(x) and is_finite(y) and math.hypot(x, y) > 1e-6
    range_angle_valid = is_finite(range_m) and is_finite(angle) and range_m > 1e-6

    if config.prefer_xy and xy_valid:
        parsed_y = -y if config.invert_y else y
        return validate_target(
            (x + config.anchor_x_offset, parsed_y + config.anchor_y_offset),
            config,
        )

    if range_angle_valid:
        angle_rad = angle * math.pi / 180.0 if config.angle_in_degrees else angle
        angle_rad += config.angle_offset_rad
        target_x = range_m * math.cos(angle_rad)
        target_y = range_m * math.sin(angle_rad)
        if config.invert_y:
            target_y = -target_y
        return validate_target(
            (target_x + config.anchor_x_offset, target_y + config.anchor_y_offset),
            config,
        )

    if xy_valid:
        parsed_y = -y if config.invert_y else y
        return validate_target(
            (x + config.anchor_x_offset, parsed_y + config.anchor_y_offset),
            config,
        )

    return None


def compute_follow_goal(
    target_xy: Tuple[float, float],
    config: FollowGoalConfig,
) -> Optional[Tuple[float, float, float]]:
    """Convert target coordinates into an MPPI local goal pose."""

    target_x, target_y = target_xy
    distance = math.hypot(target_x, target_y)
    if distance <= 1e-6 or not is_finite(distance):
        return None

    # Move along the target bearing until the robot keeps the configured gap.
    travel = distance - config.follow_distance
    if abs(travel) < config.goal_deadband:
        travel = 0.0
    travel = clamp(travel, -config.reverse_goal_limit, config.forward_goal_limit)

    unit_x = target_x / distance
    unit_y = target_y / distance
    goal_x = travel * unit_x
    goal_y = travel * unit_y
    goal_yaw = math.atan2(target_y, target_x)
    return goal_x, goal_y, goal_yaw
