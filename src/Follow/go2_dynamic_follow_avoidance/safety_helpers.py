from .geometry import clamp


def limit_delta(current: float, target: float, max_delta: float) -> float:
    if max_delta <= 0.0:
        return target
    return current + clamp(target - current, -max_delta, max_delta)
