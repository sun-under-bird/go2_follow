"""在MPPI优化之前生成跟随速度目标，参考末端与转弯提前减速。"""
import math
from .controller import clamp, wrap
from .navigation_config import MAX_NAVIGATION_SPEED, MAX_NAVIGATION_TURN, BRAKE_DECELERATION, CONTROL_HOLD_SECONDS


def stopping_speed(distance, hold_seconds):
    """求解d=v*保持时间+v²/(2a)，与末级制动检查共享减速度和延迟口径。"""
    a = BRAKE_DECELERATION
    return max(0., math.sqrt((a * hold_seconds) ** 2 + 2 * a * max(0., distance)) - a * hold_seconds)


def route_speed_reference(route, yaw, human_speed, distance, spacing, depth_age, kind, orient=False, hold_seconds=None):
    """从已校验路线给出速度需求；短已知通道仍可走，不伪造后续自由空间。"""
    if orient or len(route) < 2:
        return dict(speed=0., reason='ORIENT_OR_EMPTY', route_length_m=0.)
    lengths = [math.dist(a, b) for a, b in zip(route, route[1:])]
    cumulative = [0.]
    for length in lengths:
        cumulative.append(cumulative[-1] + length)
    headings = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(route, route[1:])]
    hold = (CONTROL_HOLD_SECONDS + min(.5, max(0., depth_age)) if hold_seconds is None else hold_seconds)
    # 人速前馈加间距误差，过近时低于人速，不引入最低前进速度。
    demand = clamp(human_speed + .6 * (distance - spacing), 0., MAX_NAVIGATION_SPEED)
    if kind == 'OBSERVING':
        demand = min(.45, demand if human_speed > .08 else .35)
    path_limit = stopping_speed(max(0., cumulative[-1] - .08), hold)
    error = abs(wrap(headings[0] - yaw))
    # 按前向投影限制追赶速度；平方余弦会把小横偏的连接误差再次放大成不必要减速。
    # 正交/背向参考仍不前进，实际转弯和停车空间另由后续联合约束校验。
    alignment_limit = MAX_NAVIGATION_SPEED * max(0., math.cos(error))
    corner_limit = MAX_NAVIGATION_SPEED
    for index in range(1, len(headings)):
        angle = abs(wrap(headings[index] - headings[index - 1]))
        if angle < .02:
            continue
        # 离转角较远时先走，再按同一制动能力逐渐降到可实现的转弯速度。
        # 裁剪旧路径时的厘米级连接线不是一个真实急弯，使用前向累计支撑长度。
        span = max(.35, min(.6, lengths[index - 1]+lengths[index]))
        curvature = angle / span
        at_corner = min(MAX_NAVIGATION_SPEED, MAX_NAVIGATION_TURN / curvature, math.sqrt(.45 / curvature))
        approach = math.sqrt(at_corner ** 2 + 2 * BRAKE_DECELERATION * max(0., cumulative[index] - .15))
        corner_limit = min(corner_limit, approach)
    limit = min(demand, path_limit, alignment_limit, corner_limit)
    return dict(speed=limit, demand=demand, path_limit=path_limit, alignment_limit=alignment_limit,
                corner_limit=corner_limit, hold_seconds=hold, route_length_m=cumulative[-1], reason='FOLLOW_SPEED_PROFILE')
