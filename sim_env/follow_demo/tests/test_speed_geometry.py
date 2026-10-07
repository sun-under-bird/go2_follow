"""覆盖平地深度误投影、短连接急转和移动参考两侧偏摆的根因反例。"""
import math
import unittest
import numpy as np
from follow_demo.camera_profile import camera_intrinsic
from follow_demo.controller import Pose
from follow_demo.local_map import RollingMap
from follow_demo.local_planner import LocalPlanner, Plan
from follow_demo.navigation import NavigationController
from follow_demo.speed_reference import route_speed_reference


class SpeedGeometryTests(unittest.TestCase):
    """检查修复后的几何一致性，同时保留障碍、未知和完整矩形扫掠约束。"""

    def floor_frame(self):
        """按理想光线与平地交点生成深度，只用作独立几何反例。"""
        fx, fy, cx, cy = camera_intrinsic(212, 120)
        rotation = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
        translation = np.array([.23, .05, .43])
        rows, _ = np.indices((120, 212))
        depth = np.divide(translation[2]*fy, rows-cy, out=np.full((120,212), np.inf), where=rows>cy)
        return depth, [fx, fy, cx, cy], rotation, translation

    def test_floor_neighbors_use_their_own_expected_depth(self):
        """3m平地相邻像素有不同深度，不能因最近深度较小而被误判为未知。"""
        grid = RollingMap(static_history=True)
        grid.confirm_start(0., 0., 1.)
        self.assertTrue(grid.integrate(*self.floor_frame(), 1.1))
        free, _, _ = grid.layers(1.1)
        self.assertTrue(grid.route_clear(free, [[0.,0.],[3.,0.]], 0.))
        self.assertFalse(grid.route_clear(free, [[0.,0.],[-2.,0.]], 0.))

    def test_visible_ground_does_not_remove_existing_high_obstacle(self):
        """仅有地面支撑不能清掉高度带未完整覆盖的高处障碍。"""
        grid = RollingMap(static_history=True)
        obstacle = grid.cell(3.,0.)
        grid.occupied[obstacle] = True
        grid.integrate(*self.floor_frame(), 1.)
        self.assertTrue(grid.occupied[obstacle])

    def test_rejoin_is_longer_and_still_cannot_cut_through_obstacle(self):
        """横偏0.2m时避免0.24m短连接急转；新连接仍要求整个矩形运动安全。"""
        core = NavigationController()
        self.addCleanup(core.close)
        core.grid.seen[:] = 1.
        core.motion = [.8,0.,0.]
        core.plan = Plan('FOLLOWING','TEST', [[0.,0.],[3.,0.]])
        pose = Pose(1.,.5,.2,0.)
        free, _, _ = core.grid.layers(1.)
        route = core.path_ahead(pose,free)
        self.assertGreater(route[1][0]-pose.x,.55)
        profile = route_speed_reference(route,0.,.8,4.,2.,.1,'FOLLOWING')
        self.assertGreater(profile['alignment_limit'],.7)
        core.grid.occupied[core.grid.cell(.8,.1)] = True
        free, _, _ = core.grid.layers(1.)
        changed = core.path_ahead(pose,free)
        self.assertFalse(changed)

    def test_open_moving_reference_stays_near_motion_line(self):
        """开阔地图不为拉长参考向左右偏走，终点满足持续前进而非无用横向距离。"""
        grid = RollingMap(static_history=True)
        grid.seen[:] = 1.
        _, allowed, clearance = grid.layers(1.)
        plan = LocalPlanner().search(grid,allowed,clearance,Pose(1.,0.,0.,0.),[5.,0.],[.8,0.],2.)
        self.assertGreater(plan.path[-1][0],2.1)
        self.assertLess(abs(plan.path[-1][1]),.12)


if __name__ == '__main__':
    unittest.main()
