"""在保存的真实观测地图上重算进展目标，隔离加权搜索预算对参考长度的影响。"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from follow_demo.local_map import RollingMap
from follow_demo.local_planner import LocalPlanner, Plan
from follow_demo.controller import FollowController


class BudgetPlanner(LocalPlanner):
    """只在离线重算中改变进展候选预算，保留原地图和矩形路径碰撞校验。"""
    def __init__(self, budget):
        """记录诊断预算；3.8为生产实现，其他数值不会写回运行参数。"""
        super().__init__()
        self.budget = budget
        self.selection = None

    def progress_candidate(self, points, travel, separation, pose, target, margin, target_speed=0.):
        """复现原进展评分，仅比较同一张图上的不同加权代价预算。"""
        offsets = points - [pose.x, pose.y]
        length = np.linalg.norm(offsets, axis=1)
        relative = np.arctan2(offsets[:, 1], offsets[:, 0]) - pose.yaw
        angles = np.abs((relative + math.pi) % (2 * math.pi) - math.pi)
        candidates = (length >= .8) & (travel <= self.budget) & (angles <= 1.05)
        candidates &= separation <= math.dist([pose.x, pose.y], target) - .35
        if not np.any(candidates):
            return None
        score = separation + .20 * travel + .15 * angles + .22 / np.maximum(.08, margin + .08)
        if self.previous.kind == 'FOLLOWING' and self.previous.path:
            score += .12 * np.linalg.norm(points - self.previous.path[-1], axis=1)
        score[~candidates] = np.inf
        selected = int(np.argmin(score))
        self.selection = dict(weighted_cost=float(travel[selected]),
                              straight_endpoint_distance_m=float(length[selected]),
                              candidate_max_distance_m=float(length[candidates].max()),
                              selected_margin_m=float(margin[selected]),
                              selected_clearance_penalty=float(.22 / max(.08, margin[selected] + .08)),
                              selected_score=float(score[selected]),
                              endpoint=points[selected].tolist())
        return selected


def main():
    """读取末帧观测快照，输出反事实搜索长度；不冒充完整闭环回放或更改生产预算。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = json.loads(args.report.read_text(encoding='utf-8'))
    rows = json.loads(args.report.with_name(args.report.stem + '-samples.json').read_text(encoding='utf-8'))
    last = report['final_navigation']
    snapshot = last['navigation']['map']
    # 使用同一末帧附近的控制位姿，不把末帧地图套到早期移动过程中。
    row = min(rows, key=lambda value: abs(value['control_t'] - last['t']))
    pose = SimpleNamespace(x=row['x'], y=row['y'], yaw=row['yaw'], t=last['t'])
    grid = RollingMap(size=snapshot['size'], resolution=snapshot['resolution'], static_history=True)
    grid.origin = np.array(snapshot['origin'])
    cells = np.frombuffer(snapshot['cells'].encode('ascii'), dtype=np.uint8).reshape((grid.size, grid.size)) - 48
    free = np.isin(cells, [1, 3, 4])
    grid.seen[free] = pose.t
    grid.occupied[:] = cells == 2
    grid.last_depth = pose.t
    actual_free, allowed, clearance = grid.layers(pose.t)
    if not np.array_equal(actual_free, free):
        raise RuntimeError('显示地图重建自由证据不一致')
    velocity = row['estimated_target_velocity']
    target = (np.asarray(row['target']) + 1.2 * np.asarray(velocity)).tolist()
    spacing = FollowController().desired_distance + .3 * min(math.hypot(*velocity), .7)
    results = []
    for budget in (3.8, 6., 10.):
        planner = BudgetPlanner(budget)
        planner.previous = Plan(kind='FOLLOWING', path=last['navigation']['path'])
        plan = planner.search(grid, allowed, clearance, pose, target, velocity, spacing)
        results.append(dict(budget=budget, kind=plan.kind, reason=plan.reason,
                            selection=planner.selection, path=plan.path,
                            reference_length_m=sum(math.dist(a, b) for a, b in zip(plan.path, plan.path[1:])),
                            rectangle_route_clear=grid.route_clear(free, plan.path, pose.yaw) if plan.path else False))
    source = Path(__file__).resolve().parents[1] / 'local_planner.py'
    evidence = dict(kind='offline_reference_budget_diagnostic',
                    limitation='仅重算开阔场景末帧附近的一张已观测地图，不是早期路径或完整闭环的反事实证明',
                    report=str(args.report), report_sha256=hashlib.sha256(args.report.read_bytes()).hexdigest(),
                    planner_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                    snapshot_t=last['t'], pose_sample_t=row['t'], pose=vars(pose),
                    target=target, velocity=velocity, spacing=spacing, free_cells=int(free.sum()), cases=results)
    args.output.write_text(json.dumps(evidence, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(results, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
