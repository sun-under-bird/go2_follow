"""与 ROS、MuJoCo 解耦的目标估计和开阔场地跟随控制核心。"""
from collections import deque
from dataclasses import dataclass
import math


def wrap(angle):
    """将角度归一到 [-π, π]，保证跨 ±π 的转向连续。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    """将有限标量限制在指定区间。"""
    return min(high, max(low, value))


@dataclass
class Pose:
    """带仿真采样时间的平面机器人位姿。"""
    t: float
    x: float
    y: float
    yaw: float
    motion: tuple = None


class PoseHistory:
    """保存短时里程计历史，为有延迟的 UWB 测量寻找采样时刻位姿。"""
    def __init__(self):
        """创建三秒的位姿历史缓存。"""
        self.values = deque()

    def add(self, pose):
        """加入新位姿；时间回跳时清空上一轮场景的缓存。"""
        if self.values and pose.t < self.values[-1].t:
            self.values.clear()
        if self.values and pose.t == self.values[-1].t:
            self.values[-1] = pose
        else:
            self.values.append(pose)
        while self.values and pose.t - self.values[0].t > 3.0:
            self.values.popleft()

    def at(self, stamp):
        """按测量时间插值，拒绝没有历史覆盖的输入，不使用当前位姿硬凑。"""
        if not self.values or stamp < self.values[0].t - 0.001:
            return None
        if stamp >= self.values[-1].t:
            return self.values[-1] if stamp - self.values[-1].t <= 0.025 else None
        for left, right in zip(self.values, list(self.values)[1:]):
            if left.t <= stamp <= right.t:
                ratio = (stamp - left.t) / max(right.t - left.t, 1e-9)
                return Pose(stamp, left.x + ratio * (right.x - left.x),
                            left.y + ratio * (right.y - left.y),
                            wrap(left.yaw + ratio * wrap(right.yaw - left.yaw)))
        return None

    def current(self, stamp, max_age=.2):
        """取不晚于控制时刻的最新测量，不把同源消息接入顺序误当时钟偏移。

        /clock与里程计独立接入，最新里程计可能暂时领先ROS控制时钟。
        领先样本保留，等控制时钟覆盖后再使用，绝不改写测量时间戳。
        同帧速度随Pose保存，避免用旧位姿配上领先样本的运动状态。
        """
        if not math.isfinite(stamp) or not math.isfinite(max_age) or max_age<0:
            return None
        for pose in reversed(self.values):
            if pose.t<=stamp:
                return pose if stamp-pose.t<=max_age else None
        return None


class FollowController:
    """根据目标运动趋势和距离区间生成连续速度，不实施最低速度抬升。"""
    def __init__(self):
        """初始化首版开阔场地参数，数值仅用于仿真，不作为实机安全标定值。"""
        self.history = PoseHistory()
        self.target = None
        self.target_velocity = [0.0, 0.0]
        self.target_stamp = None
        self.last_world_measurement = None
        self.command = [0.0, 0.0]
        self.acceleration = [0.0, 0.0]
        self.last_tick = None
        self.desired_distance = 1.8
        self.target_timeout = 0.45
        self.max_speed = 0.48
        self.max_turn = 0.85
        self.state = 'WAITING'
        self.reason = '等待目标与里程计'
        self.accepted = 0
        self.rejected = 0
        self.last_distance = None

    def observe(self, local_x, local_y, stamp):
        """把 UWB 转到采样时刻的 odom 坐标系，再做异常门控和运动趋势估计。"""
        if not all(math.isfinite(value) for value in (local_x, local_y, stamp)):
            self.rejected += 1
            return False
        pose = self.history.at(stamp)
        if pose is None or (self.target_stamp is not None and stamp <= self.target_stamp):
            self.rejected += 1
            return False
        cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
        world = [pose.x + cosine * local_x - sine * local_y,
                 pose.y + sine * local_x + cosine * local_y]
        dt = stamp - self.target_stamp if self.target_stamp is not None else None
        if dt is not None and dt < self.target_timeout:
            displacement = math.dist(world, self.last_world_measurement)
            if displacement > 0.25 + 2.0 * dt:
                self.rejected += 1
                return False
            # 世界系差分避免把机器人自身运动误当成人的速度；低通抑制 UWB 抖动。
            alpha = dt / (0.16 + dt)
            filtered = [self.target[i] + alpha * (world[i] - self.target[i]) for i in range(2)]
            velocity_alpha = dt / (0.65 + dt)
            for index in range(2):
                velocity = clamp((filtered[index] - self.target[index]) / dt, -1.5, 1.5)
                self.target_velocity[index] += velocity_alpha * (velocity - self.target_velocity[index])
            self.target = filtered
        else:
            # 长时间失联后的第一帧重新建立估计，不跨失联区间差分。
            self.target, self.target_velocity = world, [0.0, 0.0]
        self.target_stamp, self.last_world_measurement = stamp, world
        self.accepted += 1
        return True

    def stop(self, state, reason):
        """对输入失效、暂停或急停立即输出零命令，并清除平滑器残余状态。"""
        self.command, self.acceleration = [0.0, 0.0], [0.0, 0.0]
        self.state, self.reason = state, reason
        return tuple(self.command)

    def step(self, now, enabled=True, signal_valid=True, ready=True, emergency=False):
        """执行一个控制周期；正常启停限制加速度变化，输入失效优先停车。"""
        dt = 0.02 if self.last_tick is None else clamp(now - self.last_tick, 0.001, 0.10)
        self.last_tick = now
        if emergency:
            return self.stop('ESTOP', '用户急停')
        if not enabled:
            return self.stop('PAUSED', '跟随已暂停')
        if not ready:
            return self.stop('INITIALIZING', '仿真姿态准备中或姿态异常')
        if not signal_valid:
            return self.stop('TARGET_LOST', 'UWB 信号中断')
        pose = self.history.current(now)
        if pose is None:
            return self.stop('ODOM_LOST', '里程计数据过期')
        if self.target_stamp is None or not 0 <= now - self.target_stamp <= self.target_timeout:
            return self.stop('TARGET_LOST', '目标数据缺失或过期')
        age = now - self.target_stamp
        target = [self.target[i] + self.target_velocity[i] * min(age, 0.2) for i in range(2)]
        dx, dy = target[0] - pose.x, target[1] - pose.y
        distance = math.hypot(dx, dy)
        self.last_distance = distance
        if distance < 0.85:
            return self.stop('TOO_CLOSE', '目标进入近距离保护区')
        heading = wrap(math.atan2(dy, dx) - pose.yaw)
        target_speed = math.hypot(*self.target_velocity)
        radial_feedforward = (self.target_velocity[0] * dx + self.target_velocity[1] * dy) / distance
        spacing = self.desired_distance + 0.45 * min(target_speed, 0.6)
        error = distance - spacing
        # 静止目标附近使用舒适区间，避免距离噪声触发反复启停。
        correction = 0.65 * max(0.0, error - 0.10)
        forward = clamp(radial_feedforward + correction, 0.0, self.max_speed)
        if distance < spacing - 0.12 or (target_speed < 0.045 and error < 0.15):
            forward = 0.0
        forward *= max(0.0, math.cos(heading)) ** 2
        turn = clamp(1.15 * heading, -self.max_turn, self.max_turn)
        if forward == 0.0 and abs(heading) < 0.10:
            turn = 0.0
        # 首版场景保证开阔；目标到侧后方时允许转向，不适用于未知障碍空间。
        targets = [forward, turn]
        for index, (accel_limit, jerk_limit) in enumerate(((0.45, 1.8), (1.0, 4.0))):
            desired_accel = clamp((targets[index] - self.command[index]) / dt, -accel_limit, accel_limit)
            self.acceleration[index] += clamp(desired_accel - self.acceleration[index], -jerk_limit * dt, jerk_limit * dt)
            candidate = self.command[index] + self.acceleration[index] * dt
            if (targets[index] - self.command[index]) * (targets[index] - candidate) <= 0:
                candidate, self.acceleration[index] = targets[index], 0.0
            # 目标突然减速时，平滑器仍可能保留正加速度；输出硬边界优先于平滑。
            lower, upper = (0.0, self.max_speed) if index == 0 else (-self.max_turn, self.max_turn)
            bounded = clamp(candidate, lower, upper)
            if bounded != candidate:
                self.acceleration[index] = 0.0
            self.command[index] = bounded
        self.state = 'HOLDING' if abs(self.command[0]) < 0.01 and abs(self.command[1]) < 0.02 else 'FOLLOWING'
        self.reason = '已到舒适跟随距离' if self.state == 'HOLDING' else '开阔场地跟随'
        return tuple(self.command)
