"""相机地面预测的离线反例：真实内参、光心、俯仰、遮挡和缺少标定。"""
import math
import unittest
from dataclasses import FrozenInstanceError

import numpy as np

from follow_demo.local_map import RollingMap
from follow_demo.observation_geometry import (CameraGroundModel, camera_ground_projection,
                                            camera_ground_unoccluded, camera_ground_view,
                                            observation_view, check_observation_position)


class CameraObservationTests(unittest.TestCase):
    """仅验证预测的几何边界，不生成深度，不把预测当作真实地图观测。"""
    def setUp(self):
        """建立当前仿真针孔标定；实机必须使用自己的 CameraInfo 与采集 TF。"""
        focal = 60 / math.tan(math.radians(29))
        self.intrinsic = (focal, focal, 106., 60.)
        self.rotation = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
        self.model = CameraGroundModel(self.intrinsic, (120, 212), self.rotation, (.23, .05, .43))
        self.grid = RollingMap(size=101, resolution=.1, static_history=True)

    def test_ground_lower_image_edge_rejects_near_cells(self):
        """二维前视锥会计入近端地面，但水平相机在近端无法看到地面支撑。"""
        rotation, translation = self.model.transform([0., 0.], 0.)
        result = camera_ground_projection([[.80, .05], [1.30, .05]], self.intrinsic,
                                          rotation, translation, (120, 212))
        self.assertEqual(result['supported'].tolist(), [False, True])
        self.assertGreaterEqual(result['v'][0], 119)

    def test_integrate_image_border_and_range_are_identical(self):
        """一像素腐蚀边界和 .3～4.8m 的严格深度边界必须与地图积分一致。"""
        translation = np.array([0., 0., 0.])
        focal = self.intrinsic[0]
        # optical y=0，地面所有像素落在中行；水平坐标用给定像素反推。
        points = [[1., -(u - 106) / focal] for u in (0., 1., 210., 211.)]
        points += [[.3, 0.], [4.8, 0.], [.30001, 0.], [4.79999, 0.]]
        result = camera_ground_projection(points, self.intrinsic, self.rotation, translation, (120, 212))
        self.assertEqual(result['supported'].tolist(), [False, True, True, False, False, False, True, True])

    def test_from_capture_preserves_roll_pitch_and_height(self):
        """去除平面机身姿态后，再变回同一姿态应逐项恢复采集时刻真实 TF。"""
        angle, pitch = .73, -.16
        yaw = np.array([[math.cos(angle), -math.sin(angle), 0.],
                        [math.sin(angle), math.cos(angle), 0.], [0., 0., 1.]])
        tilt = np.array([[math.cos(pitch), 0., math.sin(pitch)], [0., 1., 0.],
                         [-math.sin(pitch), 0., math.cos(pitch)]])
        rotation = yaw @ tilt @ self.rotation
        translation = yaw @ np.array([.23, .05, .47]) + [2.1, 1.2, 0.]
        model = CameraGroundModel.from_capture(self.intrinsic, (120, 212), rotation,
                                               translation, [2.1, 1.2], angle)
        new_rotation, new_translation = model.transform([2.1, 1.2], angle)
        np.testing.assert_allclose(new_rotation, rotation, atol=1e-12)
        np.testing.assert_allclose(new_translation, translation, atol=1e-12)

    def test_model_is_immutable_and_copies_input_arrays(self):
        """异步搜索持有模型后，传感器不能通过旧数组引用篡改候选几何。"""
        original = self.rotation.copy()
        model = CameraGroundModel(self.intrinsic, (120, 212), original, [.23, .05, .43])
        original[:] = 0.
        np.testing.assert_equal(np.asarray(model.base_rotation), self.rotation)
        with self.assertRaises(FrozenInstanceError):
            model.intrinsic = (1., 1., 1., 1.)

    def test_invalid_calibration_is_rejected(self):
        """畸形内参、非旋转矩阵及过小图像不可默默退回假相机。"""
        with self.assertRaises(ValueError):
            CameraGroundModel((0., 1., 1., 1.), (120, 212), self.rotation, [.23, .05, .43])
        with self.assertRaises(ValueError):
            CameraGroundModel(self.intrinsic, (120, 212), np.ones((3, 3)), [.23, .05, .43])
        with self.assertRaises(ValueError):
            CameraGroundModel(self.intrinsic, (2, 212), self.rotation, [.23, .05, .43])

    def test_missing_model_produces_no_predicted_gain(self):
        """缺少传感器标定时不允许旧机身扇形继续制造观察收益。"""
        result = camera_ground_view(self.grid, np.zeros_like(self.grid.occupied), [0., 0.], 0., None)
        self.assertFalse(result['available'])
        self.assertFalse(result['visible_cells'])
        self.assertEqual(result['gain'], 0.)

    def test_first_corner_cone_can_admit_unprojectible_roi(self):
        """复现 x3.65 的旧准入：二维 ROI 重叠 100%，全部 ROI 地面却在真实像素支持外。"""
        point, heading = np.array([3.65, 1.1]), -1.22458
        self.grid.recenter(*point)
        gx, gy = self.grid.centers()
        self.grid.seen[np.hypot(gx - point[0], gy - point[1]) < .70] = 1.
        free, _, _ = self.grid.layers(2.)
        old = observation_view(self.grid, free, point, heading)
        cells = np.asarray(sorted(old['unknown_cells']))
        rotation, translation = self.model.transform(point, heading)
        projection = camera_ground_projection((cells + .5) * .1, self.intrinsic,
                                              rotation, translation, (120, 212))
        expected = set(map(tuple, cells[~projection['supported']]))
        old_check = check_observation_position(self.grid, point, heading, expected, 2.)
        self.assertTrue(old_check['valid'])
        self.assertEqual(old_check['overlap_ratio'], 1.)
        self.assertGreater(len(expected), 60)
        actual = camera_ground_view(self.grid, free, point, heading, self.model)
        self.assertFalse(actual['visible_cells'] & expected)

    def test_actual_optical_origin_changes_wall_visibility(self):
        """光心过墙端后能看见侧后地面，机身原点射线仍被墙截断。"""
        self.grid.recenter(3.5, 1.)
        gx, gy = self.grid.centers()
        self.grid.occupied[(gx >= 2.7) & (gx <= 3.3) & (gy >= -1.35) & (gy <= .35)] = True
        destination = np.array([[3.45, -.95]])
        self.assertFalse(camera_ground_unoccluded(self.grid, [3.0, .8], destination)[0])
        self.assertTrue(camera_ground_unoccluded(self.grid, [3.45, .8], destination)[0])

    def test_dda_supercover_matches_map_line_oracle(self):
        """随机光心至格中心的射线用已验证线段覆盖作独立语义对照，包括触角侧格。"""
        rng = np.random.default_rng(8301)
        self.grid.occupied[:] = rng.random(self.grid.occupied.shape) < .04
        points = np.array([self.grid.point(tuple(cell)) for cell in rng.integers(1, 100, (160, 2))])
        for origin in ([.013, .029], [.0, .0], [.05, .05]):
            actual = camera_ground_unoccluded(self.grid, origin, points)
            expected = [bool(cells) and all(not self.grid.occupied[cell] for cell in cells)
                        for cells in (self.grid.segment_cells(origin, point) for point in points)]
            self.assertEqual(actual.tolist(), expected)

    def test_corner_side_cell_blocks_even_if_diagonal_cells_are_free(self):
        """射线触角时两侧障碍不能被 DDA 的对角推进漏掉。"""
        self.grid.origin[:] = 0.
        self.grid.occupied[0, 1] = True
        result = camera_ground_unoccluded(self.grid, [.05, .05], [[.35, .35]])
        self.assertFalse(result[0])

    def test_view_prediction_does_not_write_map(self):
        """预测收益永远不能刷新已观测时间、帧数或传感器健康。"""
        before = self.grid.seen.copy()
        view = camera_ground_view(self.grid, np.zeros_like(self.grid.occupied), [0., 0.], 0., self.model)
        self.assertTrue(view['available'])
        self.assertGreater(view['covered_count'], 100)
        np.testing.assert_equal(self.grid.seen, before)
        self.assertEqual(self.grid.frames, 0)
        self.assertIsNone(self.grid.last_depth)

    def test_selected_roi_has_same_geometry_as_full_view(self):
        """局部任务 ROI 只裁剪查询范围，不能改变真实投影、遮挡和窗口外处理。"""
        free = np.zeros_like(self.grid.occupied)
        self.grid.occupied[self.grid.cell(1.75, .05)] = True
        selected = {(x, y) for x in range(5, 26) for y in range(-8, 9)}
        selected.add((999, 999))
        full = camera_ground_view(self.grid, free, [0., 0.], 0., self.model)
        roi = camera_ground_view(self.grid, free, [0., 0.], 0., self.model, selected_cells=selected)
        self.assertEqual(roi['visible_cells'], full['visible_cells'] & selected)
        self.assertEqual(roi['unknown_cells'], full['unknown_cells'] & selected)
        self.assertEqual(roi['selected_count'], len(selected))
        self.assertAlmostEqual(roi['roi_unknown_coverage'],
                               len(roi['unknown_cells']) / roi['selected_unknown_count'])
        empty = camera_ground_view(self.grid, free, [0., 0.], 0., self.model, selected_cells=set())
        self.assertEqual(empty['covered_count'], 0)

    def test_near_window_edge_cannot_wrap_negative_cell_index(self):
        """光心在窗口边的浮点容差内时，不可用 NumPy 负下标借用窗口另一侧格。"""
        self.grid.origin[:] = 0.
        self.assertFalse(camera_ground_unoccluded(self.grid, [1e-12, .15], [[.65, .15]])[0])


if __name__ == '__main__':
    unittest.main()
