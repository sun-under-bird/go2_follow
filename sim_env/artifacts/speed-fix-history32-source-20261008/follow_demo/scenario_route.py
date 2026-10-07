"""仿真目标的折线路线与循环进度；只用于合成刺激和界面，禁止输入导航器。"""
import math


def route_points(specification):
    """生成去除重复点的路线；循环路线显式闭合，有限路线保留终点。"""
    points = []
    for point in [specification.get('start', [2.0, 0.0]), *specification['route']]:
        if len(point) != 2 or not all(math.isfinite(v) and abs(v) <= 18 for v in point):
            raise ValueError('路线坐标必须是活动范围内的有限二维点')
        if not points or math.dist(points[-1], point) > 1e-9:
            points.append(list(point))
    if specification.get('loop') and math.dist(points[0], points[-1]) > 1e-9:
        points.append(points[0].copy())
    if len(points) < 2:
        raise ValueError('路线至少需要一段非零长度的直线')
    return points


class RouteWalker:
    """按累计弧长行走，不丢弃转折点剩余步长，也不在每个顶点插入停车帧。"""

    def __init__(self, specification, position=None):
        """构建路线；偏离起点时先连续返回起点，返回过程不冒充完成一圈。"""
        self.points = route_points(specification)
        self.loop = bool(specification.get('loop'))
        self.cumulative = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.cumulative.append(self.cumulative[-1] + math.dist(a, b))
        self.length = self.cumulative[-1]
        self.progress = 0.0
        self.position = list(position if position is not None else self.points[0])
        self.approach_remaining = math.dist(self.position, self.points[0])

    def locate(self, progress):
        """把弧长映射为位置和当前路段，闭环接缝处回到首段，不瞬移。"""
        distance = progress % self.length if self.loop else min(progress, self.length)
        for index, (a, b) in enumerate(zip(self.points, self.points[1:])):
            if distance < self.cumulative[index + 1] or index == len(self.points) - 2:
                ratio = (distance - self.cumulative[index]) / (self.cumulative[index + 1] - self.cumulative[index])
                return [a[axis] + ratio * (b[axis] - a[axis]) for axis in (0, 1)], index

    def advance(self, speed, dt):
        """消耗本步完整路程；同一步跨越多段或多圈也保持计时和位置连续。"""
        if not all(math.isfinite(v) and v >= 0 for v in (speed, dt)):
            raise ValueError('速度和步长必须为非负有限值')
        distance = speed * dt
        if self.approach_remaining > 1e-9:
            step = min(distance, self.approach_remaining)
            ratio = step / self.approach_remaining
            self.position = [v + ratio * (end - v) for v, end in zip(self.position, self.points[0])]
            self.approach_remaining = max(0.0, self.approach_remaining - step)
            distance -= step
        if self.approach_remaining <= 1e-9:
            self.progress += distance
            if not self.loop:
                self.progress = min(self.progress, self.length)
            self.position, _ = self.locate(self.progress)
        return self.position.copy()

    def status(self):
        """给显示和验收提供圈数、路段及累计路程，不改变跟随控制输入。"""
        _, index = self.locate(self.progress)
        return dict(loop=self.loop, laps=int(self.progress / self.length) if self.loop else 0,
                    progress_m=self.progress, lap_progress_m=self.progress % self.length if self.loop else self.progress,
                    lap_length_m=self.length, segment=index + 1, segments=len(self.points) - 1,
                    approach_remaining_m=self.approach_remaining,
                    finished=not self.loop and self.progress >= self.length)


def main():
    """为验收脚本计算覆盖指定圈数的时长，不启动任何仿真或ROS节点。"""
    import argparse
    from .scenarios import SCENARIOS
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=list(SCENARIOS), required=True)
    parser.add_argument('--laps', type=int, default=2)
    parser.add_argument('--speed', type=float, default=.8)
    args = parser.parse_args()
    if not 1 <= args.laps <= 20 or not math.isfinite(args.speed) or not .05 <= args.speed <= .80:
        parser.error('圈数必须为1至20，速度必须为0.05至0.80 m/s')
    print(math.ceil(RouteWalker(SCENARIOS[args.scenario]).length * args.laps / args.speed + 10))


if __name__ == '__main__':
    main()
