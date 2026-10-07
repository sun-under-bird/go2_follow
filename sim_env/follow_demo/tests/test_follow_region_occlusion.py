"""隔墙距离合适不等于完成跟随；未知视线也不能伪造自由空间。"""
import unittest
import numpy as np
from follow_demo.controller import Pose
from follow_demo.local_map import RollingMap
from follow_demo.local_planner import LocalPlanner


class FollowRegionOcclusionTests(unittest.TestCase):
    """检查障碍连线几何，以及绕墙过程中跟随终点的语义。"""
    def test_closed_cell_contact_and_outside_target(self):
        """平行、擦边、零长度及窗口外端点均按闭障碍格相交判断。"""
        grid = RollingMap(size=20,resolution=.1,static_history=True)
        grid.origin[:] = 0.
        grid.occupied[5,5] = True
        points = [[.1,.55],[.1,.5],[.1,.8],[.55,.55],[.8,.55]]
        blocked = LocalPlanner.wall_occluded(grid,points,[3.,.55])
        self.assertEqual(blocked.tolist(),[True,True,False,True,False])
        self.assertTrue(LocalPlanner.wall_occluded(grid,[[.55,.55]],[.55,.55])[0])

    def test_unknown_is_not_assumed_to_be_wall_or_free(self):
        """遮挡偏好不把未知当已知墙，也不修改地图自由证据。"""
        grid = RollingMap(static_history=True)
        before = grid.seen.copy()
        self.assertFalse(LocalPlanner.wall_occluded(grid,[[0.,0.]],[20.,0.])[0])
        np.testing.assert_array_equal(grid.seen,before)
        self.assertFalse(grid.segment_clear(grid.layers(1.)[0],[0.,0.],[1.,0.]))

    def test_follow_region_uses_far_side_of_known_wall(self):
        """人转到墙后时，不选墙近侧的直线距离环，已知绕行路径仍完整安全。"""
        grid = RollingMap(size=81,resolution=.1,static_history=True)
        grid.origin[:] = [-2.,-4.]
        grid.seen[:] = 1.
        x,y = grid.centers()
        grid.occupied[(x>2.8)&(x<3.2)&(y<.6)] = True
        pose = Pose(1.,2.,1.,0.)
        target = [4.,-.5]
        free,allowed,clearance = grid.layers(1.)
        before = grid.seen.copy()
        plan = LocalPlanner().search(grid,allowed,clearance,pose,target,[0.,-.5],2.1)
        self.assertEqual(plan.kind,'FOLLOWING')
        self.assertFalse(LocalPlanner.wall_occluded(grid,[plan.path[-1]],target)[0])
        self.assertTrue(grid.route_clear(free,plan.path,pose.yaw))
        np.testing.assert_array_equal(grid.seen,before)


if __name__ == '__main__':
    unittest.main()
