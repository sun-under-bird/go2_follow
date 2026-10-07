"""验证可切换轨迹意图接入；地图、UWB 门控和执行保护均使用真实代码边界。"""
import copy
import math
import unittest
from concurrent.futures import Future
from unittest.mock import patch

import numpy as np

from follow_demo.controller import Pose
from follow_demo.follow_intent import FollowIntent, TrailReferenceTracker
from follow_demo.local_map import RollingMap
from follow_demo.local_planner import LocalPlanner, Plan
from follow_demo.navigation import NavigationController
from follow_demo.tests.test_navigation_lifecycle import DeferredExecutor


class IntentReferenceTests(unittest.TestCase):
    """机器人落后、自交和历史变化时，连续投影必须保持未完成路线的先后顺序。"""

    def make_intent(self, path, robot=(0., 0.), history_length=12., velocity=(0., 0.)):
        """构造新鲜有序 UWB 历史；这些点只是人的意图，尚无自由空间证明。"""
        intent = FollowIntent(max_history_length=history_length, max_prediction=1.4)
        for index, point in enumerate(path):
            intent.update(point, 1.+index*.1, robot_position=robot,
                          velocity=velocity if index == len(path)-1 else (0., 0.))
        return intent, 1.+(len(path)-1)*.1

    def test_full_guide_retains_corner_older_than_spacing(self):
        """最近 2 m 尾段已在障碍后，完整 guide 仍让落后的狗先去旧上侧拐点。"""
        path = ((2., 0.), (2., 1.1), (4., 1.5), (4., -.5), (8., -.6))
        intent, now = self.make_intent(path)
        guide = intent.guide(now, 1.2)
        self.assertIn((2., 1.1), guide.path)
        self.assertNotIn((2., 1.1), intent.hint(now, 2.).path)
        reference = TrailReferenceTracker().build(guide, 2., (0., 0.))
        self.assertEqual(reference.next_corner, (2., 0.))
        self.assertAlmostEqual(reference.goal[0], 2.)
        self.assertAlmostEqual(reference.goal[1], .4)
        self.assertGreater(reference.remaining_arc, 5.)
        self.assertTrue(reference.virtual_stem_used)

    def test_stationary_self_cross_cannot_advance_to_later_visit(self):
        """在同一位置重复查询不会跳到人体轨迹后来回到此处的段落。"""
        intent, now = self.make_intent(((0., 0.), (2., 0.), (0., 0.), (2., 0.)))
        guide, tracker = intent.guide(now), TrailReferenceTracker()
        for _ in range(100):
            reference = tracker.build(guide, 1., (0., 0.))
            self.assertEqual(reference.projection, (0., 0.))
            self.assertAlmostEqual(reference.projection_order, 1.)
            self.assertEqual(reference.goal, (2., 0.))

    def test_fold_checkpoint_is_kept_until_robot_reaches_it(self):
        """180° 折返的前视终点不能落到更近的返回段而跳过折返点。"""
        intent, now = self.make_intent(((0., 0.), (2., 0.), (0., 0.), (2., 0.)))
        guide, tracker = intent.guide(now), TrailReferenceTracker()
        for x in (0., .5, 1., 1.5):
            reference = tracker.build(guide, 1., (x, 0.))
            self.assertEqual(reference.goal, (2., 0.))
        reference = tracker.build(guide, 1., (1.8, 0.))
        self.assertEqual(reference.next_corner, (0., 0.))
        self.assertEqual(reference.goal, (0., 0.))

    def test_real_motion_advances_to_upper_passage_before_return(self):
        """经过起始拐点后滚动参考进入上侧通道，不能直接追人的下侧最新位置。"""
        intent, now = self.make_intent(((2., 0.), (2., 1.1), (4., 1.5), (4., -.5), (8., -.6)))
        guide, tracker = intent.guide(now), TrailReferenceTracker()
        for point in ((0., 0.), (.5, 0.), (1., .1), (1.5, .3), (1.8, .2), (2., .6)):
            reference = tracker.build(guide, 2., point)
        self.assertGreater(reference.goal[1], .9)
        self.assertGreater(reference.goal[0], 2.)
        self.assertLess(reference.goal[0], 3.)
        self.assertGreater(reference.projection_order, guide.orders[1])

    def test_actual_history_trim_cannot_silently_skip_unfinished_corner(self):
        """有限历史删掉狗还没走到的开头后，失效状态持续到新意图序列重建。"""
        intent, now = self.make_intent(((2., 0.), (2., 1.)), history_length=2.)
        tracker = TrailReferenceTracker()
        self.assertEqual(tracker.build(intent.guide(now), 1., (0., 0.)).status, 'FRESH')
        intent.update((5., 1.), now+.1, robot_position=(0., 0.), velocity=(0., 0.))
        for _ in range(3):
            self.assertEqual(tracker.build(intent.guide(now+.1), 1., (0., 0.)).status, 'HISTORY_TRUNCATED')
        intent.reset()
        intent.update((5., 1.), now+.2, robot_position=(0., 0.), velocity=(0., 0.))
        self.assertEqual(tracker.build(intent.guide(now+.2), 1., (0., 0.)).status, 'FRESH')

    def test_collinear_merge_preserves_progress_without_index_dependence(self):
        """新增直行点合并掉旧数组下标后，进展通过顺序标签继续而非跳回起点。"""
        intent, now = self.make_intent(((2., 0.), (3., 0.)))
        tracker = TrailReferenceTracker()
        tracker.build(intent.guide(now), 1., (0., 0.))
        before = tracker.build(intent.guide(now), 1., (.5, 0.))
        intent.update((3.5, 0.), now+.1, velocity=(0., 0.))
        after = tracker.build(intent.guide(now+.1), 1., (.6, 0.))
        self.assertGreaterEqual(after.projection[0], before.projection[0])
        self.assertLessEqual(after.projection[0], .6+1e-8)
        self.assertEqual(after.status, 'FRESH')

    def test_virtual_orders_do_not_replace_actual_freshness_stamp(self):
        """虚拟起始点和预测端点有排序标签，但新鲜度只取真实 UWB 的采样时间。"""
        intent, now = self.make_intent(((2., 0.),), velocity=(.5, 0.))
        guide = intent.guide(now+.2, 1.2)
        self.assertLess(guide.orders[0], guide.target_stamp)
        self.assertGreater(guide.orders[-1], guide.target_stamp)
        self.assertEqual(guide.target_stamp, now)
        self.assertAlmostEqual(guide.age, .2)
        self.assertIsNone(intent.guide(now+.46, 1.2))

    def test_map_shortcut_does_not_wait_for_missed_human_corner(self):
        """狗可合法切过人的拐点时，目标仍随人前移，不能因离顶点较远而卡在旧转角。"""
        intent, now = self.make_intent(((0., 0.), (2., 0.), (2., 2.), (5., 2.)))
        guide = intent.guide(now)
        mandatory, regional = TrailReferenceTracker(), TrailReferenceTracker()
        # 这条机器人运动在全自由地图可执行；人体顶点 (2,0) 距离始终超过旧完成门槛。
        for point in ((0., 0.), (.5, .4), (1., .8), (1.5, 1.2)):
            old = mandatory.build(guide, 1., point)
            new = regional.build(guide, 1., point, preserve_corners=False)
        self.assertEqual(old.next_corner, (2., 0.))
        self.assertLess(old.goal[1], 1.)
        self.assertGreater(new.goal[1],1.5)
        self.assertEqual(new.status, 'FRESH')
        self.assertEqual(new.next_corner, (2., 0.))
        intent.update((5.5, 2.), now+.1, velocity=(0., 0.))
        advanced = regional.build(intent.guide(now+.1), 1., (1.6, 1.3), preserve_corners=False)
        self.assertGreater(advanced.goal[1],new.goal[1])

    def test_regional_goal_keeps_current_branch_when_human_runs_ahead(self):
        """人已走到后续交错障碍时，前视区域必须保留当前路段，不能跳到较近的返回分支。"""
        # 该例专门验证分支前视，必须保留16 m完整历史，不能让12 m默认裁剪改变刺激。
        intent,now=self.make_intent(((0.,0.),(4.,0.),(4.,-2.),(7.,-2.),(7.,2.),(10.,2.)),history_length=30.)
        tracker=TrailReferenceTracker()
        for robot in ((0.,0.),(1.,0.),(2.,0.),(3.,0.)):
            reference=tracker.build(intent.guide(now),1.,robot,preserve_corners=False)
        self.assertAlmostEqual(reference.goal[0],4.)
        self.assertLess(reference.goal[1],0.)
        self.assertGreater(reference.goal[1],-2.)
        # 允许越过第一个角点，但不跳到尚未绕过的后续分支。
        self.assertGreater(reference.goal[0],reference.projection[0])
        self.assertEqual(reference.status,'FRESH')

    def test_regional_goal_only_needs_retained_human_tail(self):
        """狗的旧投影被截掉时，最新真实尾段仍能定义跟随区域，不继承检查点模式的永久失效。"""
        intent, now = self.make_intent(((2.,0.),(2.,1.)),history_length=2.)
        tracker = TrailReferenceTracker()
        tracker.build(intent.guide(now),1.,(0.,0.),preserve_corners=False)
        intent.update((5.,1.),now+.1,velocity=(0.,0.))
        for _ in range(3):
            reference = tracker.build(intent.guide(now+.1),1.,(0.,0.),preserve_corners=False)
            self.assertEqual(reference.status,'FRESH')
            self.assertEqual(reference.goal,(4.,1.))
            self.assertGreater(reference.projection_gap,2.)


    def test_trimmed_anchor_reprojects_near_robot_instead_of_chasing_new_head(self):
        """历史裁剪后机器人靠近仍有记录的尾段，应重锚到近段而非八米外的新头部。"""
        intent,now=self.make_intent(((0.,0.),(4.,0.)),history_length=12.)
        tracker=TrailReferenceTracker()
        tracker.build(intent.guide(now),1.,(0.,0.),preserve_corners=False)
        intent.update((4.,8.),now+.1,velocity=(0.,0.))
        intent.update((10.,8.),now+.2,velocity=(0.,0.))
        reference=tracker.build(intent.guide(now+.2),1.,(8.,8.),preserve_corners=False)
        self.assertLess(reference.projection_gap,.01)
        self.assertGreaterEqual(reference.goal[0],8.)
        self.assertEqual(reference.status,'FRESH')


class IntentNavigationTests(unittest.TestCase):
    """试验开关改变目标语义，地图自由证据、输入失效和异步接入门槛不得改变。"""

    def test_navigation_history_keeps_unpassed_branch_in_long_detour(self):
        """空间上近的返程段不能因12米历史过短删掉尚未走完的交错障碍分支。"""
        core=NavigationController(use_follow_intent=True)
        self.addCleanup(core.close)
        points=((0.,0.),(4.,0.),(4.,-2.),(8.,-2.),(8.,4.),(0.,4.))
        for index,point in enumerate(points):
            core.follow_intent.update(point,1.+index*.1,velocity=(0.,0.))
        guide=core.follow_intent.guide(1.5)
        self.assertGreater(guide.observed_length,12.)
        self.assertEqual(guide.path[0],(0.,0.))
        self.assertEqual(core.follow_intent.max_history_length,32.)
        self.assertGreaterEqual(core.follow_intent.max_points,32./core.follow_intent.sample_distance+2)

    def make_core(self, enabled=False):
        """创建使用真实控制代码的核心，每个用例只启动并回收唯一搜索线程。"""
        core = NavigationController(use_follow_intent=enabled)
        self.addCleanup(core.close)
        return core

    def test_default_constructor_keeps_annulus_scheme(self):
        """默认不开启新方案，原目标区域和规划理由保持不变。"""
        core = self.make_core()
        core.history.add(Pose(1., 0., 0., 0.))
        self.assertTrue(core.observe(4., 0., 1.))
        self.assertIsNone(core.follow_intent)
        self.assertFalse(core.use_follow_intent)
        core.grid.seen[:] = 1.
        plan, _, _ = core.compute_plan(core.grid, Pose(1., 0., 0., 0.), np.array([4., 0.]), [0., 0.], 1.8, 1., [])
        self.assertEqual(plan.reason, 'FOLLOW_REGION_REACHABLE')
        self.assertFalse(plan.intent_reference)

    def test_only_accepted_measurements_record_raw_world_trail(self):
        """使用采样位姿转换且经过门控的世界 UWB；滤波目标或被拒异常点不能伪造轨迹。"""
        core = self.make_core(True)
        core.history.add(Pose(1., 0., 0., 0.))
        self.assertTrue(core.observe(4., 0., 1.))
        core.history.add(Pose(1.1, .1, 0., math.pi/2))
        self.assertTrue(core.observe(.2, -3.9, 1.1))
        hint = core.follow_intent.hint(1.1, 0.)
        self.assertAlmostEqual(hint.latest_target[0], 4.)
        self.assertAlmostEqual(hint.latest_target[1], .2)
        self.assertNotAlmostEqual(core.target[1], .2)
        before = core.follow_intent.observed_path()
        core.history.add(Pose(1.2, .1, 0., math.pi/2))
        self.assertFalse(core.observe(99., -99., 1.2))
        self.assertFalse(core.observe(.1, -3.9, 1.1))
        self.assertEqual(core.follow_intent.observed_path(), before)
        self.assertEqual(core.follow_intent.diagnostics()['target_stamp'], 1.1)

    def test_straight_trial_endpoint_matches_old_lookahead_region(self):
        """直行保持原有 1.2 s 规划前视，不能把开阔跟随改成极短的连续停车终点。"""
        grid = RollingMap(static_history=True)
        grid.seen[:] = 1.
        _, allowed, clearance = grid.layers(1.)
        intent = FollowIntent(max_prediction=1.4)
        intent.update((4., 0.), 1., robot_position=(0., 0.), velocity=(.5, 0.))
        reference = TrailReferenceTracker(lookahead=10.).build(intent.guide(1., 1.2), 2., (0., 0.))
        pose = Pose(1., 0., 0., 0.)
        old = LocalPlanner().search(grid, allowed, clearance, pose, [4.6, 0.], [.5, 0.], 2.)
        trial = LocalPlanner().search(grid, allowed, clearance, pose, [4.6, 0.], [.5, 0.], 2.,
                                      intent_reference=reference)
        self.assertAlmostEqual(reference.goal[0], 2.6)
        self.assertLess(math.dist(old.path[-1], trial.path[-1]), .15)
        self.assertEqual(trial.reason, 'FOLLOW_TRAIL_REFERENCE')

    def test_unknown_human_trail_never_marks_map_free(self):
        """人体走过未知区也不能扩大机器人自由地图；所有返回路线仍处于现有 allowed。"""
        grid = RollingMap(static_history=True)
        grid.confirm_start(0., 0., 1.)
        before_seen, before_occupied = grid.seen.copy(), grid.occupied.copy()
        _, allowed, clearance = grid.layers(1.)
        intent = FollowIntent()
        intent.update((3., 1.), 1., robot_position=(0., 0.), velocity=(0., 0.))
        reference = TrailReferenceTracker().build(intent.guide(1.), .5, (0., 0.))
        plan = LocalPlanner().search(grid, allowed, clearance, Pose(1., 0., 0., 0.), [3., 1.], [0., 0.], .5,
                                     intent_reference=reference)
        np.testing.assert_array_equal(grid.seen, before_seen)
        np.testing.assert_array_equal(grid.occupied, before_occupied)
        self.assertFalse(grid.permitted(allowed, *reference.goal))
        self.assertTrue(all(grid.permitted(allowed, *point) for point in plan.path))
        self.assertTrue(all(grid.segment_clear(allowed, first, second) for first, second in zip(plan.path, plan.path[1:])))

    def test_human_trail_through_wall_is_not_an_execution_path(self):
        """人的意图直穿墙时，试验规划仍绕墙，绝不能把人体折线直接发给执行器。"""
        grid = RollingMap(static_history=True)
        grid.seen[:] = 1.
        x, y = grid.centers()
        grid.occupied[(abs(x-1.4)<.15)&(abs(y)<.8)] = True
        _, allowed, clearance = grid.layers(1.)
        intent = FollowIntent()
        intent.update((4., 0.), 1., robot_position=(0., 0.), velocity=(0., 0.))
        reference = TrailReferenceTracker(lookahead=4.).build(intent.guide(1.), 1., (0., 0.))
        self.assertFalse(grid.segment_clear(allowed, *reference.path))
        plan = LocalPlanner().search(grid, allowed, clearance, Pose(1., 0., 0., 0.), [4., 0.], [0., 0.], 1.,
                                     intent_reference=reference)
        self.assertEqual(plan.kind, 'FOLLOWING')
        free,_,_ = grid.layers(1.)
        self.assertTrue(grid.route_clear(free,plan.path,0.))
        self.assertNotEqual(tuple(map(tuple, plan.path)), reference.path)
        self.assertTrue(all(grid.segment_clear(allowed, first, second) for first, second in zip(plan.path, plan.path[1:])))

    def test_invalid_signal_revokes_follow_path_and_same_tick_future(self):
        """跟随路径也必须在信号失效时撤销，旧结果不能在恢复后重新接入。"""
        executor = DeferredExecutor()
        with patch('follow_demo.navigation.ThreadPoolExecutor', return_value=executor):
            core = self.make_core(True)
        core.history.add(Pose(10., 0., 0., 0.))
        self.assertTrue(core.observe(4., 0., 10.))
        core.grid.seen[:] = 10.
        core.grid.last_depth, core.grid.confirmed = 10., True
        core.plan = Plan('FOLLOWING', 'FOLLOW_TRAIL_REFERENCE', [[0., 0.], [2., 0.]])
        core.external_active, core.external_stamp = True, 10.
        core.external_command = (.5, .1)
        reference = core.prepare_intent_reference(10., core.history.values[-1], 1.8)
        core.update_plan(core.history.values[-1], np.array([4., 0.]), [0., 0.], 1.8, 10.,
                         intent_reference=reference)
        old_future, old_plan = core.future, copy.deepcopy(core.plan)
        self.assertEqual(core.step(10., signal_valid=False), (0., 0.))
        self.assertFalse(core.plan.path)
        self.assertFalse(core.external_active)
        self.assertIsNone(core.follow_intent.guide(10.))
        old_future.set_result((old_plan, 1., 10.))
        core.history.add(Pose(10.1, 0., 0., 0.))
        self.assertTrue(core.observe(4., 0., 10.1))
        core.grid.last_depth = 10.1
        core.step(10.1)
        self.assertFalse(core.plan.path)
        self.assertIsNot(core.future, old_future)

    def test_pause_rebuilds_intent_without_repeated_reset_every_tick(self):
        """明确暂停允许修复被截断的意图，但连续暂停回调不能清掉每个新 UWB 样本。"""
        core = self.make_core(True)
        core.history.add(Pose(1., 0., 0., 0.))
        core.observe(4., 0., 1.)
        core.step(1., enabled=False)
        core.history.add(Pose(1.1, 0., 0., 0.))
        core.observe(4., 0., 1.1)
        sequence = core.follow_intent.guide(1.1).sequence
        core.step(1.1, enabled=False)
        self.assertEqual(core.follow_intent.guide(1.1).sequence, sequence)


if __name__ == '__main__':
    unittest.main()
