"""用实际 RollingMap 验证格角、边界和短接触区不会被等距采样漏掉。"""
import math
import unittest
import numpy as np
from follow_demo.local_map import RollingMap


class SegmentSafetyTests(unittest.TestCase):
    """线段覆盖与硬安全判断共享完整格语义，反向检查结果必须相同。"""

    def setUp(self):
        """构造单位格地图，便于精确表达线段接触的格集合。"""
        self.grid = RollingMap(size=7, resolution=1., radius=.1)
        self.grid.origin = np.zeros(2)
        self.allowed = np.ones((7, 7), dtype=bool)

    def assert_coverage(self, a, b, expected):
        """同时核对正向、反向和完整触碰格集合。"""
        self.assertEqual(set(self.grid.segment_cells(a, b)), set(expected))
        self.assertEqual(self.grid.segment_cells(a, b), self.grid.segment_cells(b, a))

    def test_horizontal_and_vertical_inside_cells(self):
        """水平与竖直线段覆盖每一个经过格，不依赖距离采样间隔。"""
        self.assert_coverage([.5, .5], [3.5, .5], [(0, col) for col in range(4)])
        self.assert_coverage([.5, .5], [.5, 3.5], [(row, 0) for row in range(4)])

    def test_grid_border_checks_both_sides(self):
        """沿格线运行时两侧都属于接触区，任一侧未知均不可通过。"""
        expected = [(row, col) for row in (0, 1) for col in range(3)]
        self.assert_coverage([.5, 1.], [2.5, 1.], expected)
        self.assert_coverage([1., .5], [1., 2.5], [(row, col) for row in range(3) for col in (0, 1)])
        self.allowed[0, 1] = False
        self.assertFalse(self.grid.segment_clear(self.allowed, [.5, 1.], [2.5, 1.]))

    def test_diagonal_corner_checks_adjacent_cells(self):
        """对角擦过格角必须验证两侧邻格，不允许斜穿障碍或未知。"""
        expected = [(0, 0), (0, 1), (1, 0), (1, 1), (1, 2), (2, 1), (2, 2)]
        self.assert_coverage([.5, .5], [2.5, 2.5], expected)
        self.allowed[0, 1] = False
        self.assertFalse(self.grid.segment_clear(self.allowed, [.5, .5], [2.5, 2.5]))

    def test_endpoint_on_border_includes_touched_neighbor(self):
        """终点落在格线上也不能忽略终点接触的另一格。"""
        self.assert_coverage([.5, .5], [1., .5], [(0, 0), (0, 1)])

    def test_zero_length_point_has_one_two_or_four_cells(self):
        """零长度不是空路径；内部、边界和格角分别检查实际接触格。"""
        self.assert_coverage([2.5, 3.5], [2.5, 3.5], [(3, 2)])
        self.assert_coverage([2., 3.5], [2., 3.5], [(3, 1), (3, 2)])
        self.assert_coverage([2., 3.], [2., 3.], [(2, 1), (2, 2), (3, 1), (3, 2)])
        self.allowed[2, 1] = False
        self.assertFalse(self.grid.segment_clear(self.allowed, [2., 3.], [2., 3.]))

    def test_outside_or_touching_window_edge_is_unknown(self):
        """线段越界和窗口边缘接触均返回 None，不能裁掉窗口外格。"""
        for a, b in (([.5, .5], [7.5, .5]), ([0., 2.5], [1., 2.5]),
                     ([6., 2.5], [7., 2.5]), ([.5, -.01], [.5, 2.5])):
            with self.subTest(a=a, b=b):
                self.assertIsNone(self.grid.segment_cells(a, b))
                self.assertIsNone(self.grid.segment_cells(b, a))
                self.assertFalse(self.grid.segment_clear(self.allowed, a, b))

    def test_invalid_input_is_not_safe(self):
        """非有限或不完整坐标不能成为有效空覆盖。"""
        for a in ([math.nan, .5], [math.inf, .5], [], [.5]):
            with self.subTest(a=a):
                self.assertIsNone(self.grid.segment_cells(a, [.5, .5]))

    def test_random_segments_match_independent_rectangle_intersections(self):
        """与独立逐格矩形求交对照，覆盖不同斜率和反向，不复用格线分段算法。"""
        generator = np.random.default_rng(20261004)
        for _ in range(100):
            a, b = generator.uniform(.01, 6.99, (2, 2))
            delta = b-a
            expected = set()
            for row in range(7):
                for col in range(7):
                    low, high = 0., 1.
                    for axis, minimum in ((0, col), (1, row)):
                        left, right = sorted(((minimum-a[axis])/delta[axis], (minimum+1-a[axis])/delta[axis]))
                        low, high = max(low, left), min(high, right)
                    if low <= high:
                        expected.add((row, col))
            self.assert_coverage(a, b, expected)

    def test_three_millimeter_corner_crossing_cannot_be_missed(self):
        """实际地图反例：原约三厘米采样漏掉仅三毫米长的包络禁行角格。"""
        grid = RollingMap(size=81, resolution=.1, radius=.48, static_history=True)
        grid.origin = np.array([-4.05, -4.05])
        grid.seen[:] = 1.
        gx, gy = grid.centers()
        grid.occupied[(abs(gx) <= .6+1e-8) & (abs(gy) <= .4+1e-8)] = True
        _, allowed, clearance = grid.layers(1.)
        a, b = [-3., 0.], [.40000000000000036, 1.5000000000000009]
        count = max(2, int(math.dist(a, b)/.03)+2)
        old_points = np.linspace(a, b, count)
        # 先确认反例针对原缺陷：旧采样全部通过，但实际线段接触禁行格。
        self.assertTrue(all(grid.permitted(allowed, *point) for point in old_points))
        self.assertIn((49, 32), grid.segment_cells(a, b))
        self.assertLess(float(clearance[49, 32]), grid.radius+grid.resolution*math.sqrt(.5))
        self.assertFalse(grid.segment_clear(allowed, a, b))
        self.assertFalse(grid.segment_clear(allowed, b, a))


if __name__ == '__main__':
    unittest.main()
