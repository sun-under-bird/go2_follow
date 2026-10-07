"""真实相机观察模式的局部搜索反例，旧扇区模式仍保持默认关闭。"""
import math
import unittest
from unittest.mock import patch

import numpy as np

from follow_demo.controller import Pose
from follow_demo.local_map import RollingMap
from follow_demo.local_planner import LocalPlanner, OBSERVATION_CANDIDATE_BUDGET
from follow_demo.observation_geometry import (CameraGroundModel, camera_ground_view,
                                            check_observation_position, observation_task_roi,
                                            observation_view)


class CameraPlannerTests(unittest.TestCase):
    """确认任务 ROI 的未知只影响信息评分，不能绕过已知自由空间路径条件。"""
    def setUp(self):
        """准备有起始净空但目标在盲区的地图，强制搜索进入观察用途。"""
        self.grid = RollingMap(static_history=True)
        self.grid.confirm_start(0., 0., 1.)
        self.pose = Pose(1., 0., 0., 0.)
        focal = 60 / math.tan(math.radians(29))
        self.model = CameraGroundModel((focal, focal, 106., 60.), (120, 212),
                                      [[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]],
                                      (.23, .05, .43))

    def search(self):
        """通过正常入口搜索，目标真值和障碍模型都没有输入规划器。"""
        _, allowed, clearance = self.grid.layers(1.)
        return LocalPlanner().search(self.grid, allowed, clearance, self.pose,
                                     [-4., 0.], [0., 0.], 1.8)

    def test_camera_opt_in_requires_model_without_cone_fallback(self):
        """旧模式有观察收益，真实模式未获取标定时必须报告不可用。"""
        self.grid.camera_observation = False
        self.assertEqual(self.search().kind, 'OBSERVING')
        self.grid.camera_observation, self.grid.camera_model = True, None
        result = self.search()
        self.assertEqual(result.kind, 'WAITING')
        self.assertEqual(result.reason, 'CAMERA_GEOMETRY_UNAVAILABLE')
        self.assertFalse(result.path)

    def test_camera_option_does_not_change_existing_known_follow_corridor(self):
        """相机观察开关只影响信息任务，已经有证据的跟随搜索仍使用原 allowed。"""
        self.grid.seen[:] = 1.
        _, allowed, clearance = self.grid.layers(1.)
        old = LocalPlanner().search(self.grid, allowed, clearance, self.pose, [4., 0.], [.5, 0.], 1.8)
        self.grid.camera_observation, self.grid.camera_model = True, None
        new = LocalPlanner().search(self.grid, allowed, clearance, self.pose, [4., 0.], [.5, 0.], 1.8)
        self.assertEqual(old.kind, 'FOLLOWING')
        self.assertEqual(new.path, old.path)
        self.assertEqual(new.reason, old.reason)

    def test_camera_region_uses_entire_unknown_task_roi(self):
        """保留整个下一段包络任务，区域每个点按同一实际光心/像素支持准入。"""
        self.grid.camera_observation, self.grid.camera_model = True, self.model
        free, allowed, _ = self.grid.layers(1.)
        result = self.search()
        self.assertEqual(result.kind, 'OBSERVING')
        expected = observation_task_roi(self.grid, free, result.path[-1], result.target)
        self.assertEqual(set(map(tuple, result.observation_cells)), expected)
        self.assertTrue(result.observation_region)
        for point in result.observation_region:
            status = check_observation_position(self.grid, point, result.look_yaw, expected, 1.)
            self.assertTrue(status['valid'])
        for a, b in zip(result.path, result.path[1:]):
            self.assertTrue(self.grid.segment_clear(allowed, a, b))
        self.assertGreater(len(expected), 0)

    def test_camera_scoring_budget_is_at_most_four_views_per_candidate(self):
        """任务 ROI 评分预算保持原 32×4，区域核验另外独立执行。"""
        self.grid.camera_observation, self.grid.camera_model = True, self.model
        with patch('follow_demo.local_planner.camera_ground_view', wraps=camera_ground_view) as view:
            with patch.object(LocalPlanner, 'observation_region', return_value=[[0., 0.]]):
                result = self.search()
            self.assertEqual(result.kind, 'OBSERVING')
            self.assertLessEqual(view.call_count, OBSERVATION_CANDIDATE_BUDGET * 4)
            self.assertGreater(view.call_count, 0)
            self.assertTrue(all(call.kwargs.get('selected_cells') is not None for call in view.call_args_list))

    def test_near_ground_blind_roi_is_not_admitted_by_default_camera_check(self):
        """复现旧扇区重叠百分百，但真实图像无法支持该地面任务的准入反例。"""
        point, heading = np.array([3.65, 1.1]), -1.22458
        self.grid.recenter(*point)
        self.grid.seen[:] = -np.inf
        gx, gy = self.grid.centers()
        self.grid.seen[np.hypot(gx - point[0], gy - point[1]) < .70] = 1.
        free, _, _ = self.grid.layers(1.)
        old = observation_view(self.grid, free, point, heading)
        actual = camera_ground_view(self.grid, free, point, heading, self.model)
        expected = old['unknown_cells'] - actual['visible_cells']
        self.assertTrue(check_observation_position(self.grid, point, heading, expected, 1.)['valid'])
        self.grid.camera_observation, self.grid.camera_model = True, self.model
        status = check_observation_position(self.grid, point, heading, expected, 1.)
        self.assertFalse(status['valid'])
        self.assertEqual(status['overlap_count'], 0)

    def test_missing_model_also_rejects_actual_position(self):
        """搜索之后模型丢失，实际准入不得继续使用旧预测视野。"""
        self.grid.camera_observation, self.grid.camera_model = True, None
        status = check_observation_position(self.grid, [0., 0.], 0., {(15, 0)}, 1.)
        self.assertFalse(status['valid'])
        self.assertEqual(status['reason'], 'CAMERA_GEOMETRY_UNAVAILABLE')

    def test_roi_is_rectangle_sweep_with_unknown_only_and_no_map_mutation(self):
        """ROI 按包络几何产生，已知自由/已知占用不能混入待揭示任务。"""
        self.grid.seen[:] = -np.inf
        self.grid.occupied[self.grid.cell(.55, .05)] = True
        self.grid.seen[self.grid.cell(.65, .05)] = 1.
        free, _, _ = self.grid.layers(1.)
        before = self.grid.seen.copy()
        selected = observation_task_roi(self.grid, free, [0., 0.], [4., 0.])
        points = (np.asarray(sorted(selected)) + .5) * self.grid.resolution
        self.assertTrue(np.all(points[:,0] >= -.35-self.grid.resolution/2-1e-8))
        self.assertTrue(np.all(points[:,0] <= 1.35+self.grid.resolution/2+1e-8))
        self.assertTrue(np.all(abs(points[:,1]) <= .16+self.grid.resolution/2+1e-8))
        self.assertNotIn((5, 0), selected)
        self.assertNotIn((6, 0), selected)
        self.assertNotIn((20, 0), selected)
        np.testing.assert_equal(self.grid.seen, before)

    def test_roi_direction_changes_task_not_execution_map(self):
        """绕墙的侧向询问仍被保留；改变 ROI 朝向不得增加可行驶格。"""
        free, allowed, _ = self.grid.layers(1.)
        before = allowed.copy()
        left = observation_task_roi(self.grid, free, [0., 0.], [0., 4.])
        right = observation_task_roi(self.grid, free, [0., 0.], [4., 0.])
        self.assertTrue(left)
        self.assertTrue(right)
        self.assertNotEqual(left, right)
        np.testing.assert_equal(self.grid.layers(1.)[1], before)

    def test_roi_retains_window_outside_and_stops_at_near_target(self):
        """窗口外缺失格不得裁掉；目标只相距 .2m 时不凭空生成 1m 的后续中心段。"""
        self.grid.seen[:] = -np.inf
        free = np.zeros_like(self.grid.occupied)
        edge = [float(self.grid.origin[0] + (self.grid.size - .5) * self.grid.resolution), 0.]
        selected = observation_task_roi(self.grid, free, edge, [edge[0] + 3., 0.])
        offset = np.rint(self.grid.origin / self.grid.resolution).astype(int)
        self.assertTrue(any(x >= offset[0] + self.grid.size for x, _ in selected))
        near = observation_task_roi(self.grid, free, [0., 0.], [.2, 0.])
        self.assertLess(max((x + .5) * self.grid.resolution for x, _ in near), .8)
        self.assertFalse(observation_task_roi(self.grid, free, [0., 0.], [0., 0.]))


if __name__ == '__main__':
    unittest.main()
