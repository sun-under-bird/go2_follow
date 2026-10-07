"""用真实障碍地图验证后处理不会重新挤窄搜索选出的避障通道。"""
import unittest
import numpy as np
from follow_demo.controller import Pose
from follow_demo.local_map import RollingMap
from follow_demo.local_planner import LocalPlanner


class PathComfortTests(unittest.TestCase):
    """检查构造墙角的完整覆盖和净空，避免只断言函数返回值。"""
    def test_simplification_preserves_room_around_box(self):
        """原版拉直把 0.45 m 舒适净空压至 0.03 m；宽地图中应保留至少 0.20 m。"""
        grid = RollingMap(size=81,static_history=True)
        grid.origin = np.array([-4.05,-4.05])
        grid.seen[:] = 1.
        x,y = grid.centers()
        grid.occupied[(abs(x)<=.5+1e-8)&(abs(y)<=.5+1e-8)] = True
        _,allowed,clearance = grid.layers(1.)
        plan = LocalPlanner().search(grid,allowed,clearance,Pose(1.,-3.,0.,0.),[5.,0.],[.5,0.],2.)
        self.assertTrue(plan.path)
        covered = set()
        for a,b in zip(plan.path,plan.path[1:]):
            self.assertTrue(grid.segment_clear(allowed,a,b))
            covered.update(grid.segment_cells(a,b))
        margin = min(clearance[row,col] for row,col in covered)-grid.radius-grid.resolution*np.sqrt(.5)
        self.assertGreaterEqual(margin,.20-1e-7)

    def test_comfort_rule_does_not_disconnect_narrow_known_corridor(self):
        """原通道不足额外舒适宽度时仍可前进，不能通过硬膨胀伪造安全停车成绩。"""
        grid = RollingMap(static_history=True)
        x,y = grid.centers()
        grid.seen[(abs(y)<.61)&(x>-2)&(x<5)] = 1.
        _,allowed,clearance = grid.layers(1.)
        plan = LocalPlanner().search(grid,allowed,clearance,Pose(1.,0.,0.,0.),[8.,0.],[.5,0.],1.8)
        self.assertEqual(plan.kind,'FOLLOWING')
        self.assertGreater(plan.path[-1][0],.8)
        self.assertTrue(all(grid.segment_clear(allowed,a,b) for a,b in zip(plan.path,plan.path[1:])))


if __name__ == '__main__':
    unittest.main()
