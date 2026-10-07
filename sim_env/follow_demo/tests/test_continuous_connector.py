"""用连续交错障碍的真实失败地图验证连续位姿接入，不添加场景真值自由格。"""
import json
from pathlib import Path
import unittest
import numpy as np
from follow_demo.controller import Pose
from follow_demo.local_map import RollingMap
from follow_demo.local_planner import LocalPlanner


class ContinuousConnectorTests(unittest.TestCase):
    """当前矩形安全但八朝向格无法直接接入时，先用已知空间内的连续前缀连接。"""
    def test_actual_safe_pose_can_connect_without_in_place_grid_alignment(self):
        """第二障碍前的原始地图必须能给出安全进展，过程中不改变地图证据。"""
        fixture = json.loads((Path(__file__).parent/'fixtures/rectangle-connector-consecutive-20261005.json').read_text(encoding='utf-8'))
        grid = RollingMap(size=fixture['size'],resolution=fixture['resolution'],static_history=True)
        grid.origin = np.array(fixture['origin'])
        pose = Pose(*fixture['pose'])
        image = np.fromiter(map(int,fixture['cells']),dtype=np.uint8).reshape(grid.size,grid.size)
        grid.seen[np.isin(image,[1,3,4])] = pose.t
        grid.occupied[image==2] = True
        before = grid.seen.copy()
        free,allowed,clearance = grid.layers(pose.t)
        self.assertTrue(grid.pose_clear(free,pose.x,pose.y,pose.yaw))
        plan = LocalPlanner().search(grid,allowed,clearance,pose,fixture['target'],[0,0],1.8)
        self.assertEqual(plan.kind,'FOLLOWING',plan.reason)
        self.assertTrue(grid.route_clear(free,plan.path,pose.yaw))
        self.assertGreater(plan.path[-1][0]-pose.x,.5)
        np.testing.assert_array_equal(grid.seen,before)


if __name__ == '__main__':
    unittest.main()
