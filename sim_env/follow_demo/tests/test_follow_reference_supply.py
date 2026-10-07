"""覆盖移动参考的零长度反例及停人后的保持边界，不放宽路径安全条件。"""
import math
import unittest
from unittest.mock import patch

from follow_demo.controller import Pose
from follow_demo.local_map import RollingMap
from follow_demo.local_planner import LocalPlanner
from follow_demo.tests import test_navigation


class FollowReferenceSupplyTests(unittest.TestCase):
    """局部距离带能到达时也需要持续参考，未知与狭窄通道仍由原地图限制。"""

    def test_tangent_moving_target_does_not_select_immediate_success_goal(self):
        """人沿距离带切向快走时，狗所在格不能因零路程成本反复结束跟踪。"""
        grid = RollingMap(static_history=True)
        grid.seen[:] = 1.
        _, allowed, clearance = grid.layers(1.)
        pose = Pose(1.,0.,0.,0.)
        plan = LocalPlanner().search(grid, allowed, clearance, pose, [2.,-.6], [0.,-.5], 2.15)
        self.assertEqual(plan.kind,'FOLLOWING')
        self.assertGreater(math.dist(plan.path[0],plan.path[-1]),.8)
        self.assertTrue(all(grid.segment_clear(allowed,a,b) for a,b in zip(plan.path,plan.path[1:])))

    def test_short_known_channel_remains_usable(self):
        """足够长的参考只是偏好，地图只支持短段时不能转去未知区或拒绝这段路线。"""
        grid = RollingMap(static_history=True)
        gx,gy = grid.centers()
        grid.seen[(gx>-.9)&(gx<1.1)&(abs(gy)<.65)] = 1.
        _,allowed,clearance = grid.layers(1.)
        pose = Pose(1.,0.,0.,0.)
        plan = LocalPlanner().search(grid,allowed,clearance,pose,[2.3,0.],[.5,0.],2.15)
        self.assertEqual(plan.kind,'FOLLOWING')
        self.assertLess(math.dist(plan.path[0],plan.path[-1]),.8)
        self.assertTrue(all(grid.segment_clear(allowed,a,b) for a,b in zip(plan.path,plan.path[1:])))

    def test_unknown_sight_allows_stationary_hold_without_moving(self):
        """部分视线未知不能迫使狗离开已知安全站位；状态明确回报未确认视线。"""
        core = test_navigation.NavigationTests.ready_controller(self,2.04)
        core.tracking_requested,core.external_active = True,True
        core.external_stamp,core.external_command = 1.,(.5,0.)
        core.grid.seen[core.grid.cell(1.4,0.)] = -math.inf
        self.assertEqual(core.step(1.),(0.,0.))
        self.assertEqual(core.code,'FOLLOW_DISTANCE_UNOBSERVED')
        self.assertEqual(core.state,'HOLDING')
        self.assertFalse(core.tracking_requested)
        self.assertFalse(core.external_active)

    def test_stationary_hold_does_not_override_unsafe_residual_motion(self):
        """人已停在合适距离内，实际残余运动仍须经过同一制动检查。"""
        core = test_navigation.NavigationTests.ready_controller(self,2.04)
        with patch.object(core,'braking_safe',return_value=(False,[])):
            self.assertEqual(core.step(1.),(0.,0.))
        self.assertEqual(core.code,'BRAKING_SPACE')
        self.assertNotEqual(core.state,'HOLDING')

    def test_braking_stop_preserves_optimizer_but_never_emits_candidate(self):
        """有效路径下制动归零不取消优化；本拍已有高速候选也不能被执行。"""
        core = test_navigation.NavigationTests.ready_controller(self,4.)
        core.external_active,core.external_stamp,core.external_command = True,1.,(.8,.4)
        with patch.object(core,'braking_safe',return_value=(False,[])):
            self.assertEqual(core.step(1.),(0.,0.))
        self.assertEqual(core.code,'BRAKING_SPACE')
        self.assertTrue(core.tracking_requested)
        self.assertEqual(core.speed_reference['speed'],0.)

    def test_invalid_path_or_lost_input_cannot_keep_braking_optimizer(self):
        """保留热启动不扩大授权；路径切断及UWB失效仍撤销跟踪。"""
        core = test_navigation.NavigationTests.ready_controller(self,4.)
        core.grid.occupied[core.grid.cell(1.,0.)] = True
        with patch.object(core,'braking_safe',return_value=(False,[])):
            self.assertEqual(core.step(1.),(0.,0.))
        self.assertFalse(core.tracking_requested)
        core.grid.occupied[:] = False
        core.tracking_requested = True
        self.assertEqual(core.step(1.,signal_valid=False),(0.,0.))
        self.assertFalse(core.tracking_requested)


if __name__ == '__main__':
    unittest.main()
