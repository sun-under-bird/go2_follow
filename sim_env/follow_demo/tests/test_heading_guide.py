"""验证角度诊断、停人看向的滞回、旋转安全与移动路径优先。"""
import math
import unittest
from follow_demo.controller import Pose
from follow_demo.heading_guide import heading_diagnostics,path_heading
from follow_demo.navigation import NavigationController
from follow_demo.local_planner import Plan


class HeadingGuideTests(unittest.TestCase):
    """看向人只能在停人且距离合适时发生，不能重新引入绕障到一半硬转身。"""
    def core(self):
        """提供宽阔真实自由证据，结束时回收任务池。"""
        core = NavigationController()
        self.addCleanup(core.close)
        core.grid.seen[:] = 1.
        core.grid.last_depth,core.grid.confirmed = 1.,True
        return core

    def test_person_direction_and_path_direction_are_separate(self):
        """绕障时机身已对齐路径，人与机身仍有90°差，不能把它称为路径跟踪误差。"""
        diagnostic = heading_diagnostics(Pose(1,0,0,0),[0,2],[[0,0],[2,0]],[0,.5],(0,0),(0,0,0))
        self.assertAlmostEqual(diagnostic['target_error'],math.pi/2)
        self.assertAlmostEqual(diagnostic['path_error'],0)

    def test_wrapped_error_does_not_jump_at_pi(self):
        """跨越±180°只产生2°误差。"""
        angle = math.radians(-179)
        diagnostic = heading_diagnostics(Pose(1,0,0,math.radians(179)),[math.cos(angle),math.sin(angle)],[],[0,0],(0,0),(0,0,0))
        self.assertAlmostEqual(math.degrees(diagnostic['target_error']),2)

    def test_zero_length_path_has_no_invented_heading(self):
        """重复路径点不能被 atan2(0,0) 伪装成朝东。"""
        self.assertIsNone(path_heading([[0,0],[0,0]],[0,0]))

    def test_stationary_target_waits_for_stability(self):
        """目标短暂减速不立即抢占路径，稳定0.4秒后才开始看向。"""
        core = self.core()
        free,_,_ = core.grid.layers(1)
        self.assertFalse(core.prepare_facing(Pose(1,0,0,-.4),[2,0],free,1))
        self.assertTrue(core.prepare_facing(Pose(1.5,0,0,-.4),[2,0],free,1.5))
        self.assertEqual(core.plan.kind,'FACING')

    def test_hysteresis_does_not_repeatedly_start_small_turns(self):
        """初始小角度保持，已启动转向则使用更小的结束阈值。"""
        core = self.core()
        core.stationary_since = 0.
        free,_,_ = core.grid.layers(1)
        self.assertFalse(core.prepare_facing(Pose(1,0,0,-.08),[2,0],free,1))
        self.assertTrue(core.prepare_facing(Pose(1,0,0,-.3),[2,0],free,1))
        self.assertTrue(core.prepare_facing(Pose(1,0,0,-.08),[2,0],free,1))
        self.assertFalse(core.prepare_facing(Pose(1,0,0,-.04),[2,0],free,1))

    def test_facing_cannot_rotate_through_unknown_corner(self):
        """原地首尾姿态自由，中途扫角未知仍禁止看向。"""
        core = self.core()
        core.stationary_since = 0.
        free,_,_ = core.grid.layers(1)
        free = free.copy()
        free[core.grid.cell(.23,.23)] = False
        self.assertFalse(core.prepare_facing(Pose(1,0,0,0),[0,2],free,1))
        self.assertEqual(core.face_diagnostic['status'],'ROTATION_BLOCKED')

    def test_facing_clamps_forward_to_zero_even_with_mppi_forward_candidate(self):
        """看向任务收到带前进量的候选，也不能离开保持位置向人冲过去。"""
        core = self.core()
        pose = Pose(1,0,0,-.3)
        core.history.add(pose)
        core.target,core.target_stamp,core.target_velocity = [2,0],1.,[0,0]
        core.stationary_since = 0.
        free,_,_ = core.grid.layers(1)
        self.assertTrue(core.prepare_facing(pose,[2,0],free,1))
        core.external_active,core.external_stamp,core.external_command = True,1.,(.5,.4)
        command = core.step(1)
        self.assertEqual(command[0],0)
        self.assertTrue(core.tracking_requested)
        self.assertEqual(core.phase,'FACING')

    def test_moving_target_exits_facing_and_searches_path(self):
        """人重新走动，撤销原地朝向目标，重新由通道搜索引导运动。"""
        core = self.core()
        core.history.add(Pose(1,0,0,0))
        core.target,core.target_stamp,core.target_velocity = [2,1],1.,[.5,0]
        core.plan = Plan('FACING','FOLLOW_FACE_TARGET',path=[[0,0]],look_yaw=.4)
        core.face_active = True
        core.step(1)
        self.assertNotEqual(core.plan.kind,'FACING')
        self.assertFalse(core.face_active)


if __name__ == '__main__':
    unittest.main()
