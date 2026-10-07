"""验证候选前速与残余转向/侧移组合，以及完整停车段的硬安全边界。"""
import unittest
import numpy as np
from follow_demo.controller import Pose
from follow_demo.local_map import RollingMap
from follow_demo.navigation import NavigationController
from follow_demo.footprint import RectangleFootprint
from follow_demo.navigation_config import execution_hold_seconds


class ExecutionGuardTests(unittest.TestCase):
    """用极小方形隔离残余运动组合；实际0.7×0.32矩形另由 test_rectangle_footprint 覆盖。"""

    def core_and_map(self, resolution=.02):
        """创建真实地图与控制核心，检查结束后释放尚未启动任务的搜索池。"""
        core = NavigationController()
        self.addCleanup(core.close)
        grid = RollingMap(size=101,resolution=resolution,static_history=True,
                          footprint=RectangleFootprint(1e-8,1e-8))
        grid.origin = np.full(2, -50.5*resolution)
        grid.last_depth = 1.
        core.grid = grid
        pose = Pose(1., 0., 0., 0.)
        return core, grid, pose

    def one_motion_safe(self, core, pose, motion, allowed):
        """用指定实测状态检查一条停车参考，用于分辨各组合导致的拒收。"""
        core.motion = motion
        return core.braking_safe(pose, (motion[0], motion[2]), allowed, actual=True)[0]

    def test_candidate_acceleration_and_old_turn_can_leave_both_safe_paths(self):
        """旧慢速左转和请求快速右转分别安全，快速前进加旧左转却进入禁行区。"""
        core, grid, pose = self.core_and_map()
        gx, gy = grid.centers()
        allowed = ~((gx > .12) & (gy > .03))
        self.assertTrue(self.one_motion_safe(core, pose, (.10, 0., .70), allowed))
        self.assertTrue(self.one_motion_safe(core, pose, (.50, 0., -.70), allowed))
        self.assertFalse(self.one_motion_safe(core, pose, (.50, 0., .70), allowed))
        core.motion = (.10, 0., .70)
        self.assertFalse(core.braking_safe(pose, (.50, -.70), allowed)[0])

    def test_midpoint_turn_branch_is_not_replaced_by_only_two_edges(self):
        """两侧转弯都能绕过前方窄禁区，中间直行响应仍会经过禁区。"""
        core, grid, pose = self.core_and_map()
        gx, gy = grid.centers()
        allowed = ~((gx > .22) & (gx < .32) & (abs(gy) < .012))
        self.assertTrue(self.one_motion_safe(core, pose, (.05, 0., .70), allowed))
        self.assertTrue(self.one_motion_safe(core, pose, (.50, 0., .70), allowed))
        self.assertTrue(self.one_motion_safe(core, pose, (.50, 0., -.70), allowed))
        self.assertFalse(self.one_motion_safe(core, pose, (.50, 0., 0.), allowed))
        core.motion = (.05, 0., .70)
        self.assertFalse(core.braking_safe(pose, (.50, -.70), allowed)[0])

    def test_candidate_must_keep_measured_side_drift(self):
        """旧慢速侧移和无侧移快进分别安全，快进叠加实测侧移会扫到另一侧边界。"""
        core, grid, pose = self.core_and_map()
        gx, gy = grid.centers()
        allowed = ~((gx > .10) & (gy > .05))
        self.assertTrue(self.one_motion_safe(core, pose, (.05, .20, 0.), allowed))
        self.assertTrue(self.one_motion_safe(core, pose, (.50, 0., 0.), allowed))
        self.assertFalse(self.one_motion_safe(core, pose, (.50, .20, 0.), allowed))
        core.motion = (.05, .20, 0.)
        self.assertFalse(core.braking_safe(pose, (.50, 0.), allowed)[0])

    def test_zero_command_does_not_make_residual_motion_disappear(self):
        """全零请求仍需为当前速度预留停车空间，不能以原地点安全代替停车轨迹。"""
        core, grid, pose = self.core_and_map()
        gx, _ = grid.centers()
        allowed = gx < .20
        self.assertTrue(grid.permitted(allowed, pose.x, pose.y))
        self.assertFalse(self.one_motion_safe(core, pose, (.50, 0., 0.), allowed))
        self.assertFalse(core.braking_safe(pose, (0., 0.), allowed)[0])
        # 即使前速已经为零，实测侧移仍会离开停车区。
        _, gy = grid.centers()
        core.motion = (0., .20, 0.)
        self.assertFalse(core.braking_safe(pose, (0., 0.), gy < .05)[0])

    def test_request_slower_than_actual_does_not_shorten_parking_distance(self):
        """请求减速尚未生效时，停车起始前速至少保留实测前速。"""
        core, grid, pose = self.core_and_map()
        gx, _ = grid.centers()
        allowed = gx < .20
        self.assertTrue(self.one_motion_safe(core, pose, (.05, 0., 0.), allowed))
        core.motion = (.50, 0., 0.)
        self.assertFalse(core.braking_safe(pose, (.05, 0.), allowed)[0])

    def test_physics_step_segment_cannot_jump_over_forbidden_cell(self):
        """0.04秒积分的两个端点都安全，中间短禁行格仍必须拒收。"""
        core, grid, pose = self.core_and_map(resolution=.005)
        allowed = np.ones((grid.size, grid.size), dtype=bool)
        allowed[50, 51] = False
        first_position = [.20*.04, 0.]
        self.assertTrue(grid.permitted(allowed, pose.x, pose.y))
        self.assertTrue(grid.permitted(allowed, *first_position))
        self.assertFalse(grid.segment_clear(allowed, [pose.x, pose.y], first_position))
        self.assertFalse(self.one_motion_safe(core, pose, (.20, 0., 0.), allowed))

    def test_wide_known_space_remains_available(self):
        """组合校验不是一律禁止换向或侧移，空间足够时应保留候选。"""
        core, grid, pose = self.core_and_map()
        allowed = np.ones((grid.size, grid.size), dtype=bool)
        core.motion = (.10, .03, .70)
        self.assertTrue(core.braking_safe(pose, (.50, -.70), allowed)[0])
        self.assertTrue(core.braking_safe(pose, (0., 0.), allowed)[0])

    def test_static_projection_age_is_not_actuator_hold_time(self):
        """旧深度已转到odom的静态证据不重复增加执行时间；非静态模式保留旧增量。"""
        self.assertEqual(execution_hold_seconds(.4,True),.25)
        self.assertEqual(execution_hold_seconds(.4,False),.65)
        core,grid,pose = self.core_and_map()
        grid.last_depth = .6
        free = np.ones((grid.size,grid.size),dtype=bool)
        self.assertTrue(core.braking_trajectory(pose,.4,.2,0.,free)[0])
        self.assertEqual(core.trajectory_diagnostic['hold_s'],.25)
        grid.static_history = False
        self.assertTrue(core.braking_trajectory(pose,.4,.2,0.,free)[0])
        self.assertEqual(core.trajectory_diagnostic['hold_s'],.65)


if __name__ == '__main__':
    unittest.main()
