"""冻结优化前的覆盖实现，逐段核对标量快路，并提供离线短段计时入口。"""
import argparse
import contextlib
import io
import json
import math
from pathlib import Path
import platform
import statistics
import sys
import time
import unittest

import numpy as np

from follow_demo.local_map import RollingMap


COMPARISONS = 0


def reference_segment_cells(grid, a, b):
    """原样冻结优化前的 NumPy 算法，独立验证覆盖集合及浮点边界语义。"""
    try:
        endpoints = np.asarray([a, b], dtype=float)
    except (TypeError, ValueError):
        return None
    if endpoints.shape != (2, 2) or not np.isfinite(endpoints).all():
        return None
    points = (endpoints-grid.origin)/grid.resolution
    if np.any(points < 0.) or np.any(points > grid.size):
        return None
    floors = np.floor(points).astype(int)
    fractions = points-floors
    if (np.array_equal(floors[0], floors[1]) and np.all(fractions > 1e-9)
            and np.all(fractions < 1.-1e-9) and np.all(floors[0] < grid.size)):
        return [(int(floors[0, 1]), int(floors[0, 0]))]
    delta = points[1]-points[0]
    times = [np.array([0., 1.])]
    for axis in range(2):
        if delta[axis] == 0.:
            continue
        low, high = min(points[:, axis]), max(points[:, axis])
        borders = np.arange(math.ceil(low), math.floor(high)+1, dtype=float)
        crossing = (borders-points[0, axis])/delta[axis]
        times.append(crossing[(crossing >= 0.) & (crossing <= 1.)])
    times = np.unique(np.concatenate(times))
    checks = np.concatenate((times, (times[:-1]+times[1:])*.5))
    positions = points[0]+checks[:, None]*delta
    rounded = np.rint(positions)
    touches = np.abs(positions-rounded) <= 1e-9
    upper = np.where(touches, rounded, np.floor(positions)).astype(int)
    lower = upper-touches.astype(int)
    columns = np.concatenate((upper[:, 0], lower[:, 0], upper[:, 0], lower[:, 0]))
    rows = np.concatenate((upper[:, 1], upper[:, 1], lower[:, 1], lower[:, 1]))
    if (np.any(columns < 0) or np.any(columns >= grid.size)
            or np.any(rows < 0) or np.any(rows >= grid.size)):
        return None
    flat = np.unique(rows*grid.size+columns)
    return list(zip((flat//grid.size).tolist(), (flat % grid.size).tolist()))


def reference_segment_clear(grid, allowed, a, b):
    """冻结原逐段安全判断，确保优化不改变 allowed 的判定结果。"""
    cells = reference_segment_cells(grid, a, b)
    if cells is None:
        return False
    indices = np.asarray(cells, dtype=int)
    return bool(np.all(allowed[indices[:, 0], indices[:, 1]]))


class SegmentFastEquivalenceTests(unittest.TestCase):
    """按实际地图尺度和整数格边界核对快路、长段备用分支及拒收条件。"""

    def setUp(self):
        """准备单位格、实际十厘米格和非整数窗口原点三个地图。"""
        self.maps = []
        for resolution, origin in ((1., (0., 0.)), (.1, (-6.1, -6.1)),
                                   (.02, (-1.01, -.77))):
            grid = RollingMap(size=121, resolution=resolution)
            grid.origin = np.asarray(origin, dtype=float)
            self.maps.append(grid)

    def compare(self, grid, a, b, label='', reverse=True):
        """严格逐项对照旧结果；不合并近似交点，不允许用宽松集合掩盖差异。"""
        global COMPARISONS
        for first, second in ((a, b), (b, a)) if reverse else ((a, b),):
            expected = reference_segment_cells(grid, first, second)
            actual = grid.segment_cells(first, second)
            COMPARISONS += 1
            self.assertEqual(actual, expected, msg=f'{label}: {first!r} -> {second!r}')

    def world(self, grid, point):
        """把测试格坐标转换到世界坐标，覆盖实际减原点和除分辨率误差。"""
        return (np.asarray(point, dtype=float)*grid.resolution+grid.origin).tolist()

    def test_deterministic_random_short_segments(self):
        """两万条短段及其反向覆盖同格、跨格、不同斜率和实际地图变换。"""
        generator = np.random.default_rng(20261005)
        for index in range(20000):
            grid = self.maps[index % len(self.maps)]
            a = generator.uniform(.01, grid.size-.01, 2)
            b = a+generator.uniform(-1.2, 1.2, 2)
            self.compare(grid, self.world(grid, a), self.world(grid, b), '随机短段')

    def test_deterministic_random_long_segments(self):
        """三千条任意长段及其反向核对长段备用分支和窗口外拒收。"""
        generator = np.random.default_rng(19890612)
        for index in range(3000):
            grid = self.maps[index % len(self.maps)]
            a, b = generator.uniform(-.1, grid.size+.1, (2, 2))
            self.compare(grid, self.world(grid, a), self.world(grid, b), '随机长段')

    def test_integer_grid_lines_corners_and_tiny_offsets(self):
        """整格、角点和容差两侧微扰均逐段对照；保留几乎同时的两轴交点。"""
        epsilons = (0., -1e-12, 1e-12, -1e-10, 1e-10, -1e-9, 1e-9,
                    -1.00001e-9, 1.00001e-9, -2e-9, 2e-9)
        for grid in self.maps:
            for value in (0., 1., 2., 60., 119., 120., 121.):
                for epsilon in epsilons:
                    coordinate = value+epsilon
                    for a, b in (((coordinate, 58.25), (coordinate, 60.75)),
                                 ((58.25, coordinate), (60.75, coordinate)),
                                 ((coordinate, 59.), (coordinate, 59.)),
                                 ((coordinate, coordinate), (coordinate, coordinate)),
                                 ((value-.5, value-.5), (value+.5, value+.5+epsilon)),
                                 ((value-.5, value+.5), (value+.5, value-.5+epsilon)),
                                 ((value-.5, coordinate), (value, coordinate)),
                                 ((value, coordinate), (value+.5, coordinate))):
                        self.compare(grid, self.world(grid, a), self.world(grid, b), '格线/角点微扰')
            # 两轴交点时间相差若干浮点位；不能在排序时按 epsilon 归并它们。
            for epsilon in (np.spacing(.5), -np.spacing(.5), np.spacing(60.), -np.spacing(60.)):
                self.compare(grid, self.world(grid, (59.5, 59.5)),
                             self.world(grid, (60.5, 60.5+epsilon)), '浮点相邻交点')

    def test_invalid_inputs_and_window_edges(self):
        """非有限、形状错误和窗口边界继续拒收，零长点继续保留接触格。"""
        for grid in self.maps:
            valid = self.world(grid, (60.5, 60.5))
            for invalid in ([], [1.], [1., 2., 3.], None, '12', {0: 1., 1: 2.},
                            [[1.], [2.]], np.ones((2, 1)), np.ones((1, 2)),
                            [np.array([1.]), np.array([2.])],
                            [math.nan, 1.], [math.inf, 1.], [1., -math.inf]):
                self.compare(grid, invalid, valid, '无效输入')
            for point in ((0., 60.), (121., 60.), (60., 0.), (60., 121.),
                          (-1e-12, 60.), (121.+1e-12, 60.), (.5, .5), (60., 60.)):
                a = self.world(grid, point)
                self.compare(grid, a, a, '零长与窗口边界')

    def test_allowed_decisions_match_without_safety_relaxation(self):
        """随机禁行层逐段对照，并逐一禁止格线与角点覆盖格以验证及时拒收。"""
        generator = np.random.default_rng(9052026)
        for grid in self.maps:
            allowed = generator.random((grid.size, grid.size)) > .15
            for _ in range(1000):
                a = generator.uniform(.01, grid.size-.01, 2)
                b = a+generator.uniform(-1.2, 1.2, 2)
                first, second = self.world(grid, a), self.world(grid, b)
                self.assertEqual(grid.segment_clear(allowed, first, second),
                                 reference_segment_clear(grid, allowed, first, second))
            for a, b in (((59.5, 60.), (60.5, 60.)), ((60., 59.5), (60., 60.5)),
                         ((59.5, 59.5), (60.5, 60.5)), ((60., 60.), (60., 60.))):
                first, second = self.world(grid, a), self.world(grid, b)
                cells = reference_segment_cells(grid, first, second)
                for cell in cells:
                    allowed = np.ones((grid.size, grid.size), dtype=bool)
                    allowed[cell] = False
                    self.assertFalse(grid.segment_clear(allowed, first, second), msg=str(cell))


def benchmark(samples=10000, repeats=5):
    """在同一进程交替计时固定短段的旧、新覆盖与安全检查，不启动服务。"""
    grid = RollingMap(size=121, resolution=.1)
    allowed = np.ones((grid.size, grid.size), dtype=bool)
    generator = np.random.default_rng(20261005)
    starts = generator.uniform(2., 119., (samples, 2))
    ends = starts+generator.uniform(-.35, .35, (samples, 2))
    segments = [(tuple(a*.1+grid.origin), tuple(b*.1+grid.origin))
                for a, b in zip(starts, ends)]
    functions = {
        'reference_cells': lambda a, b: reference_segment_cells(grid, a, b),
        'optimized_cells': grid.segment_cells,
        'reference_clear': lambda a, b: reference_segment_clear(grid, allowed, a, b),
        'optimized_clear': lambda a, b: grid.segment_clear(allowed, a, b),
    }
    timings = {name: [] for name in functions}
    for function in functions.values():
        for a, b in segments[:200]:
            function(a, b)
    # 交替顺序降低 CPU 温度与后台调度的顺序偏差，不在计时区生成测试数据。
    for repeat in range(repeats):
        order = list(functions) if repeat % 2 == 0 else list(reversed(functions))
        for name in order:
            started = time.perf_counter()
            for a, b in segments:
                functions[name](a, b)
            timings[name].append(time.perf_counter()-started)
    medians = {name: statistics.median(values) for name, values in timings.items()}
    same_cell = sum(len(reference_segment_cells(grid, a, b)) == 1 for a, b in segments)
    return {
        'samples_per_round': samples,
        'rounds': repeats,
        'seed': 20261005,
        'same_cell_segments': same_cell,
        'cross_cell_segments': samples-same_cell,
        'max_axis_length_cells': .35,
        'python': sys.version,
        'numpy': np.__version__,
        'platform': platform.platform(),
        'seconds_per_round': timings,
        'median_seconds': medians,
        'median_microseconds_per_segment': {name: elapsed*1e6/samples for name, elapsed in medians.items()},
        'cells_speedup': medians['reference_cells']/medians['optimized_cells'],
        'clear_speedup': medians['reference_clear']/medians['optimized_clear'],
    }


def main():
    """默认运行等价性测试；显式计时模式保存完整离线结果和测试日志。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark', action='store_true')
    parser.add_argument('--samples', type=int, default=10000)
    parser.add_argument('--repeats', type=int, default=5)
    parser.add_argument('--output', type=Path)
    options = parser.parse_args()
    if not options.benchmark:
        unittest.main(argv=[sys.argv[0]])
        return
    log = io.StringIO()
    with contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
        suite = unittest.defaultTestLoader.loadTestsFromTestCase(SegmentFastEquivalenceTests)
        result = unittest.TextTestRunner(verbosity=2).run(suite)
    report = {'tests_run': result.testsRun, 'tests_passed': result.wasSuccessful(),
              'covered_segment_comparisons': COMPARISONS, 'allowed_decision_comparisons': 3000}
    if result.wasSuccessful():
        report['benchmark'] = benchmark(options.samples, options.repeats)
    summary = json.dumps(report, ensure_ascii=False, indent=2)
    if options.output:
        options.output.parent.mkdir(parents=True, exist_ok=True)
        options.output.write_text(summary+'\n', encoding='utf-8')
        options.output.with_suffix('.log').write_text(log.getvalue()+'\n'+summary+'\n', encoding='utf-8')
    print(log.getvalue(), end='')
    print(summary)
    if not result.wasSuccessful():
        raise SystemExit(1)


if __name__ == '__main__':
    main()
