"""验证观察区域去重，防止实际停点与参考点不同导致同视向无收益任务反复启动。"""
import copy
import math
import unittest
from concurrent.futures import Future
from unittest.mock import MagicMock, patch
import numpy as np
from follow_demo import observation_geometry
from follow_demo.controller import Pose, wrap
from follow_demo.local_map import RollingMap
from follow_demo.local_planner import LocalPlanner, Plan
from follow_demo.navigation import NavigationController
from follow_demo.observation import ObservationSession


class ObservationAttemptTests(unittest.TestCase):
    """覆盖记录、搜索和候选接入三个入口，旧三项记录仍维持原距离规则。"""
    def setUp(self):
        """复现实际停在区域近侧、参考点相距超过 0.2 m 的开阔场景反例。"""
        self.actual = [5.47, -.03]
        self.reference = [5.75, -.05]
        self.heading = -.79
        self.region = [[x, y] for x in (5.45, 5.55, 5.65, 5.75) for y in (-.05, .05)]
        self.attempt = [*self.actual, self.heading, copy.deepcopy(self.region)]
        self.assertGreater(math.dist(self.actual, self.reference), .2)

    def navigation_candidate(self, point, heading, attempt=None, resolution=.1):
        """返回受控的有效异步候选，完全不创建搜索线程或读取仿真服务。"""
        with patch('follow_demo.navigation.ThreadPoolExecutor', return_value=MagicMock()):
            core = NavigationController()
        self.addCleanup(core.close)
        core.grid = RollingMap(resolution=resolution, static_history=True)
        core.grid.recenter(*point)
        core.grid.seen[:] = 10.
        core.grid.confirmed, core.grid.last_depth = True, 10.
        core.history.add(Pose(10., point[0] - .1, point[1], 0.))
        self.assertTrue(core.observe(4., 0., 10.))
        core.observation.tried = [copy.deepcopy(self.attempt if attempt is None else attempt)]
        core.last_plan = 10.1
        candidate = Plan('OBSERVING', 'GOAL_UNOBSERVED',
                         [[core.history.values[-1].x, core.history.values[-1].y], list(point)], heading,
                         observation_region=[list(point)])
        core.future = Future()
        core.future.set_result((candidate, 20., 10.))
        core.update_plan(core.history.values[-1], np.asarray(core.target), [0., 0.], 1.8, 10.1)
        return core, candidate

    def test_same_world_region_and_heading_are_attempted_without_expansion(self):
        """同一世界格的参考点和非中心点被排除，区域外及不同朝向仍可尝试。"""
        attempted = observation_geometry.observation_attempted
        self.assertTrue(attempted([self.attempt], self.reference, self.heading))
        self.assertTrue(attempted([self.attempt], [5.74, -.02], self.heading))
        # 区域最右格止于 x=5.8；不能给区域再加 0.2 m 距离圆，误封锁相邻新视点。
        self.assertFalse(attempted([self.attempt], [5.86, -.05], self.heading))
        self.assertFalse(attempted([self.attempt], self.reference, self.heading + .8))
        wrapped = [*self.actual, math.pi - .04, copy.deepcopy(self.region)]
        self.assertTrue(attempted([wrapped], self.reference, -math.pi + .04))

    def test_legacy_record_and_map_resolution_keep_distinct_semantics(self):
        """旧记录继续按 0.2 m 邻域判断；区域记录按提供的地图分辨率判断。"""
        attempted = observation_geometry.observation_attempted
        legacy = self.attempt[:3]
        self.assertTrue(attempted([legacy], [5.57, -.03], self.heading))
        self.assertFalse(attempted([legacy], self.reference, self.heading))
        self.assertFalse(attempted([legacy], self.actual, self.heading + .8))
        coarse = [.35, .15, 0., [[.35, .15]]]
        self.assertTrue(attempted([coarse], [.26, .06], 0., resolution=.2))
        self.assertFalse(attempted([coarse], [.26, .06], 0., resolution=.1))

    def test_finish_preserves_original_region_for_later_exclusion(self):
        """无收益终止记录原观察区域，实际停点不能缩小任务身份或放过原参考点。"""
        grid = RollingMap(static_history=True)
        grid.recenter(*self.actual)
        grid.seen[:] = 10.
        plan = Plan('OBSERVING', 'GOAL_UNOBSERVED', [self.actual, self.reference], self.heading,
                    observation_region=copy.deepcopy(self.region))
        session = ObservationSession()
        pose = Pose(10., *self.actual, self.heading)
        session.begin(plan, pose, 10., grid)
        active_region = session.plan.observation_region
        session.finish('OBSERVATION_NO_RELEVANT_GAIN', Pose(10.5, *self.actual, self.heading), grid)
        self.assertEqual(session.tried[-1][:3], self.attempt[:3])
        self.assertGreaterEqual(len(session.tried[-1]), 4)
        self.assertEqual(session.tried[-1][3], self.region)
        # 终止证据应独立保存，旧任务持有的列表不能事后扩大已尝试区域。
        active_region.append([5.95, -.05])
        self.assertEqual(session.tried[-1][3], self.region)
        self.assertTrue(session.excluded(self.reference, self.heading))
        self.assertFalse(session.excluded([5.86, -.05], self.heading))
        self.assertFalse(session.excluded(self.reference, self.heading + .8))

    def test_planner_uses_region_exclusion_but_keeps_different_view(self):
        """一个位置曾按原视向观察后，搜索可选择该位置的新视向，不能重复原视向。"""
        grid = RollingMap(static_history=True)
        grid.recenter(5.4, 0.)
        self.assertTrue(grid.confirm_start(5.4, 0., 10.))
        _, allowed, clearance = grid.layers(10.)
        cells = np.argwhere(allowed)
        points = grid.origin + (cells[:, ::-1] + .5) * grid.resolution
        matching = np.flatnonzero(np.linalg.norm(points - self.reference, axis=1) < 1e-6)
        self.assertEqual(len(matching), 1)
        pose, target = Pose(10., 5.4, 0., 0.), [8., -2.3]
        # 仅固定位置集合，未知收益、朝向评分和安全区域仍走真实搜索逻辑。
        def matching_reachable(grid,cells,points,*args):
            """候选索引属于当前姿态可达点集，不能挪用二维并集的旧索引。"""
            return np.flatnonzero(np.linalg.norm(points-self.reference,axis=1)<1e-6)
        with patch.object(LocalPlanner,'observation_candidates',side_effect=matching_reachable):
            first = LocalPlanner().search(grid, allowed, clearance, pose, target, [0., 0.], 1.8)
            self.assertEqual(first.kind, 'OBSERVING')
            self.assertLess(abs(wrap(first.look_yaw - self.heading)), .1)
            second = LocalPlanner().search(grid, allowed, clearance, pose, target, [0., 0.], 1.8,
                                           [self.attempt])
        self.assertEqual(second.kind, 'OBSERVING')
        self.assertLess(math.dist(second.path[-1], self.reference), 1e-6)
        self.assertGreaterEqual(abs(wrap(second.look_yaw - self.heading)), .35)

    def test_future_cannot_restart_tried_region_but_new_views_remain_allowed(self):
        """排除记录产生后才到达的旧候选，也必须在 begin 与 revision 修改前被拒绝。"""
        cases = (
            ('同区域参考点', self.reference, self.heading, False),
            ('同区域实际点', [5.74, -.02], self.heading, False),
            ('区域外', [5.86, -.05], self.heading, True),
            ('不同视向', self.reference, self.heading + .8, True),
        )
        for name, point, heading, accepted in cases:
            with self.subTest(candidate=name):
                core, candidate = self.navigation_candidate(point, heading)
                if accepted:
                    self.assertEqual(core.plan, candidate)
                    self.assertEqual(core.observation.sequence, 1)
                    self.assertEqual(core.plan_revision, 1)
                else:
                    self.assertFalse(core.plan.path)
                    self.assertIsNone(core.observation.plan)
                    self.assertEqual(core.observation.sequence, 0)
                    self.assertEqual(core.plan_revision, 0)

    def test_future_exclusion_uses_current_map_resolution(self):
        """接入判定必须使用控制器地图的分辨率，不能依赖 helper 的 0.1 m 默认值。"""
        attempt = [.35, .15, 0., [[.35, .15]]]
        core, _ = self.navigation_candidate([.26, .06], 0., attempt=attempt, resolution=.2)
        self.assertFalse(core.plan.path)
        self.assertIsNone(core.observation.plan)
        self.assertEqual(core.observation.sequence, 0)
        self.assertEqual(core.plan_revision, 0)


if __name__ == '__main__':
    unittest.main()
