"""验证目标暂不可达时沿已知通道进展，保持未知、障碍及观察转向的原安全边界。"""
import math
import unittest
import numpy as np
from follow_demo.controller import Pose
from follow_demo.local_map import RollingMap
from follow_demo.local_planner import LocalPlanner


class CorridorProgressTests(unittest.TestCase):
    """构造完整地图反例，区分能继续通行和能停下原地观察。"""
    def setUp(self):
        """统一使用朝向前方的实际位置，不依赖目标真值提供地图。"""
        self.grid = RollingMap(static_history=True)
        self.pose = Pose(1.,0.,0.,0.)
        self.gx,self.gy = self.grid.centers()

    def reveal_rectangle(self, left, right, half_width):
        """只声明指定矩形已被传感器观测，矩形外保持未知。"""
        self.grid.seen[(self.gx > left) & (self.gx < right) & (abs(self.gy) < half_width)] = 1.

    def search(self, target):
        """从真实自由及包络地图搜索远目标，返回地图以便独立校验整段执行路径。"""
        _,allowed,clearance = self.grid.layers(1.)
        plan = LocalPlanner().search(self.grid,allowed,clearance,self.pose,target,[0.,0.],1.8)
        return plan,allowed,clearance

    def assert_safe_path(self, plan, allowed):
        """独立密采样每条简化线段，确保安全终点之间也没有跨越未知或障碍。"""
        self.assertTrue(plan.path)
        self.assertEqual(plan.path[0],[self.pose.x,self.pose.y])
        for first,second in zip(plan.path,plan.path[1:]):
            count = max(2,math.ceil(math.dist(first,second)/.025)+1)
            for point in np.linspace(first,second,count):
                self.assertTrue(self.grid.permitted(allowed,*point),msg=f'路径进入非安全格：{point}')

    def test_far_target_uses_observed_corridor_before_exploring(self):
        """目标远超相机地图时，已有前方通道仍产生持续移动参考，不为舒适距离不可达而停转。"""
        self.reveal_rectangle(-1.2,4.,1.2)
        plan,allowed,_ = self.search([20.,0.])
        self.assertEqual(plan.kind,'FOLLOWING')
        self.assertEqual(plan.reason,'KNOWN_CORRIDOR_PROGRESS')
        self.assertGreater(plan.path[-1][0],.8)
        self.assertLess(plan.path[-1][0],4.)
        self.assert_safe_path(plan,allowed)
        self.assertFalse(plan.observation_region)

    def test_unknown_gap_cannot_connect_observed_far_island(self):
        """未知带两侧都有观测也不能跳过去，进展路线只能在起点连通区内结束。"""
        self.reveal_rectangle(-1.2,5.,1.2)
        gap = (self.gx > 1.9) & (self.gx < 2.2)
        self.grid.seen[gap] = -np.inf
        plan,allowed,_ = self.search([20.,0.])
        self.assertEqual(plan.reason,'KNOWN_CORRIDOR_PROGRESS')
        self.assertLess(max(point[0] for point in plan.path),1.9)
        self.assertTrue(allowed[self.grid.cell(3.,0.)])
        self.assert_safe_path(plan,allowed)

    def test_complete_wall_keeps_progress_on_near_side(self):
        """横向墙截断全部通道时，即使墙后已知，也不能借远目标产生穿墙进展。"""
        self.grid.seen[:] = 1.
        self.grid.occupied[(self.gx > 1.9) & (self.gx < 2.2)] = True
        plan,allowed,_ = self.search([20.,0.])
        self.assertEqual(plan.reason,'KNOWN_CORRIDOR_PROGRESS')
        self.assertLess(max(point[0] for point in plan.path),1.9)
        self.assertTrue(allowed[self.grid.cell(3.,0.)])
        self.assert_safe_path(plan,allowed)

    def test_unknown_rear_target_does_not_create_reverse_progress(self):
        """身后目标没有已知路线时不能盲退，观察路线本身也必须留在已知安全区。"""
        self.reveal_rectangle(-1.2,4.,1.2)
        plan,allowed,_ = self.search([-20.,0.])
        self.assertFalse(allowed[self.grid.cell(-2.,0.)])
        self.assertNotEqual(plan.reason,'KNOWN_CORRIDOR_PROGRESS')
        self.assertNotEqual(plan.kind,'FOLLOWING')
        if plan.path:
            self.assert_safe_path(plan,allowed)

    def test_narrow_corridor_can_progress_without_turning_candidates(self):
        """已满足通行包络但没有额外漂移余量的通道可以前进，仍不得用作原地观察区。"""
        self.reveal_rectangle(-2.,5.,.31)
        plan,allowed,clearance = self.search([8.,0.])
        self.assertTrue(allowed[self.grid.cell(0.,0.)])
        self.assertEqual(plan.kind,'FOLLOWING')
        self.assertEqual(plan.reason,'KNOWN_CORRIDOR_PROGRESS')
        self.assert_safe_path(plan,allowed)
        cells = np.argwhere(allowed)
        points = self.grid.origin+(cells[:,::-1]+.5)*self.grid.resolution
        travel = np.linalg.norm(points-[self.pose.x,self.pose.y],axis=1)
        separation = np.linalg.norm(points-[8.,0.],axis=1)
        candidates = LocalPlanner.observation_candidates(self.grid,cells,points,travel,separation,
                                                         clearance,self.pose)
        self.assertEqual(len(candidates),0)
        self.assertFalse(plan.observation_region)


if __name__ == '__main__':
    unittest.main()
