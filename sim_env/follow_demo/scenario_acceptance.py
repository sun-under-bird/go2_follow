"""循环场景的实际路线验收；目标圈数不能替代机器狗的物理进度。"""
import math


def ordered_gate_progress(samples, points, radius=2.1):
    """依次跨过路段中间截面再返回起点区域，允许正常跟随中的横向切弯。"""
    gates = [[(a[axis] + b[axis]) / 2 for axis in (0, 1)] for a, b in zip(points, points[1:])]
    # 起点把同一直线拆成首段和短闭合段，短段中点不是独立障碍通行门。
    # 收圈检查回到起点区域，允许在已知自由通道内切弯，仍需先依次经过所有前段。
    gates[-1] = list(points[-1])
    passed = []
    index = 0
    for row in samples:
        segment = index % len(gates)
        gate = gates[segment]
        delta = [points[segment + 1][axis] - points[segment][axis] for axis in (0, 1)]
        length = math.hypot(*delta)
        displacement = [row['x'] - gate[0], row['y'] - gate[1]]
        along = sum(a*b for a,b in zip(delta,displacement)) / length
        across = abs(delta[0]*displacement[1]-delta[1]*displacement[0]) / length
        # 前段必须真的越过中间截面，不能提前接近一个圆域就记作通过。
        # 横向允许约一个正常跟随距离的偏移；这只是成绩判定，不放宽碰撞、间距和停顿门槛。
        passed_gate = (along >= 0 and across <= radius) if segment < len(gates)-1 else math.hypot(*displacement) <= radius
        if passed_gate:
            # 每个样本最多计一个门，避免密集折线同一位置被一次计成多段。
            passed.append(dict(t=row['t'], gate=index % len(gates) + 1, lap=index // len(gates) + 1,
                               position=[row['x'], row['y']]))
            index += 1
    return dict(completed_laps=index // len(gates), passed_gates=index, gates_per_lap=len(gates),
                next_gate=index % len(gates) + 1, gate_lateral_tolerance_m=radius, gates=gates, events=passed)


def loop_completion(samples, points, required_laps, report):
    """要求人和狗都完成指定圈数，末段仍在跟随；碰撞与流畅性另外独立判定。"""
    robot = ordered_gate_progress(samples, points)
    human = samples[-1].get('target_route') or {}
    tail = [row for row in samples if samples[-1]['t'] - row['t'] <= 2.0]
    checks = dict(physical_progress=report['checks']['physical_progress'],
                  final_window_observed=len(tail) >= 2 and tail[-1]['t'] - tail[0]['t'] >= 1.5,
                  target_completed_required_laps=human.get('laps', 0) >= required_laps,
                  robot_completed_required_laps=robot['completed_laps'] >= required_laps,
                  target_keeps_walking=all(row['target_mode'] == 'route' for row in samples),
                  final_follow_distance_reasonable=all(.85 <= row['distance'] <= 3.5 for row in tail))
    return dict(met=all(checks.values()), checks=checks, far_side_required=False, loop=True,
                required_laps=required_laps, target_completed_laps=human.get('laps', 0), robot=robot,
                criteria=dict(gate_lateral_tolerance_m=2.1, closure_gate='route_start', final_distance_range_m=[.85, 3.5],
                              reason='按真实位置依次越过各段中间截面，再返回起点区域；允许通道内切弯，不能用人圈数或仅接近起点替代'))
