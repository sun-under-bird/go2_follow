import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class TargetFilterConfig:
    min_valid_samples: int = 3
    max_invalid_samples: int = 4
    hold_sec: float = 0.5
    smoothing_alpha: float = 0.35
    max_jump_m: float = 1.0
    max_speed_mps: float = 2.5
    min_distance_m: float = 0.15
    max_distance_m: float = 8.0


@dataclass(frozen=True)
class TargetFilterResult:
    xy: Optional[Tuple[float, float]]
    tracking: bool
    status: str


class TargetFilter:
    def __init__(self, config: TargetFilterConfig):
        self.config = config
        self.filtered_xy: Optional[Tuple[float, float]] = None
        self.last_valid_time: Optional[float] = None
        self.last_sample_time: Optional[float] = None
        self.valid_count = 0
        self.invalid_count = 0
        self.tracking = False
        self.status = "waiting"

    def update(
        self,
        raw_xy: Optional[Tuple[float, float]],
        raw_valid: bool,
        now_sec: float,
        invalid_reason: str = "invalid",
    ) -> TargetFilterResult:
        self.last_sample_time = now_sec

        if not raw_valid or raw_xy is None:
            return self._mark_invalid(now_sec, invalid_reason)

        x, y = raw_xy
        distance = math.hypot(x, y)
        if not math.isfinite(x) or not math.isfinite(y):
            return self._mark_invalid(now_sec, "invalid: non-finite")
        if distance < self.config.min_distance_m:
            return self._mark_invalid(now_sec, "invalid: too close")
        if distance > self.config.max_distance_m:
            return self._mark_invalid(now_sec, "invalid: too far")

        if self.filtered_xy is not None:
            jump = math.hypot(x - self.filtered_xy[0], y - self.filtered_xy[1])
            if jump > self.config.max_jump_m:
                return self._mark_invalid(now_sec, f"hold: jump {jump:.2f}m")
            if self.last_valid_time is not None:
                dt = max(1e-3, now_sec - self.last_valid_time)
                speed = jump / dt
                if speed > self.config.max_speed_mps:
                    return self._mark_invalid(now_sec, f"hold: speed {speed:.2f}m/s")

        alpha = min(1.0, max(0.0, self.config.smoothing_alpha))
        if self.filtered_xy is None or alpha >= 1.0:
            self.filtered_xy = (x, y)
        else:
            fx, fy = self.filtered_xy
            self.filtered_xy = (fx + alpha * (x - fx), fy + alpha * (y - fy))

        self.last_valid_time = now_sec
        self.valid_count += 1
        self.invalid_count = 0
        self.tracking = self.valid_count >= max(1, self.config.min_valid_samples)
        self.status = "tracking" if self.tracking else f"acquiring: {self.valid_count}"
        return TargetFilterResult(self.filtered_xy, self.tracking, self.status)

    def current(self, now_sec: float, sample_timeout_sec: float) -> TargetFilterResult:
        if self.last_sample_time is None:
            return TargetFilterResult(None, False, "waiting")
        if now_sec - self.last_sample_time > sample_timeout_sec:
            self.tracking = False
            return TargetFilterResult(None, False, "stale: UWB timeout")
        if not self.tracking or self.filtered_xy is None:
            return TargetFilterResult(self.filtered_xy, False, self.status)
        if self.last_valid_time is None or now_sec - self.last_valid_time > self.config.hold_sec:
            self.tracking = False
            return TargetFilterResult(None, False, "stale: target hold expired")
        return TargetFilterResult(self.filtered_xy, True, self.status)

    def _mark_invalid(self, now_sec: float, reason: str) -> TargetFilterResult:
        self.invalid_count += 1
        if not self.tracking:
            self.valid_count = 0
        if self.invalid_count > self.config.max_invalid_samples:
            self.valid_count = 0
            self.tracking = False
            self.status = reason
            return TargetFilterResult(None, False, self.status)

        if (
            self.tracking
            and self.filtered_xy is not None
            and self.last_valid_time is not None
            and now_sec - self.last_valid_time <= self.config.hold_sec
        ):
            self.status = reason
            return TargetFilterResult(self.filtered_xy, True, self.status)

        self.tracking = False
        self.valid_count = 0
        self.status = reason
        return TargetFilterResult(self.filtered_xy, False, self.status)
