"""覆盖物理 trial 暴露的噪声假拐点，防止静态无噪声直线测试掩盖流畅度退化。"""
import math
import random
import unittest

from follow_demo.follow_intent import FollowIntent, IntentGuide, TrailReferenceTracker


def adjacent_turn(guide, cumulative, index):
    """复现失败版本用相邻短段判角点的口径，只用于证明反例能触发旧根因。"""
    path = guide.path
    before = tuple(path[index][axis]-path[index-1][axis] for axis in range(2))
    after = tuple(path[index+1][axis]-path[index][axis] for axis in range(2))
    denominator = math.hypot(*before)*math.hypot(*after)
    if denominator <= 1e-12:
        return None
    return sum(first*second for first, second in zip(before, after))/denominator, ()


class IntentCornerEvidenceTests(unittest.TestCase):
    """角点需要持续空间支持；人真实的直角和折返仍必须得到确认。"""

    def guide(self, path, stamp=30., prediction=None):
        """构造已记录几何的只读 guide；未提供预测时，所有点均是实际意图片段。"""
        orders = tuple(float(index) for index in range(len(path)))
        latest = path[-1] if prediction is None else path[-2]
        predicted = latest if prediction is None else path[-1]
        return IntentGuide(tuple(path), orders, 1, latest, predicted, stamp, 0.,
                           0. if prediction is None else .6, FollowIntent._length(path), False)

    def test_physical_open_report_false_corner_is_rejected(self):
        """实际失败报告 t=22.47 的 8 cm 短段产生约 23° 假拐点，新证据应判近直线。"""
        path = ((5.318201354047435, -.020283385182236505),
                (7.063393400279176, -.01884531139758147),
                (7.137815320523899, .01263976514178794),
                (7.707003842673748, .01463059114918543))
        guide, tracker = self.guide(path), TrailReferenceTracker()
        cumulative = tracker.cumulative(path)
        old, _ = adjacent_turn(guide, cumulative, 1)
        new, evidence = tracker.supported_turn(guide, cumulative, 1)
        self.assertGreater(math.acos(old), tracker.corner_angle)
        self.assertLess(math.acos(new), tracker.corner_angle)
        self.assertAlmostEqual(dict(evidence)['before_arc_m'], .3)
        self.assertAlmostEqual(dict(evidence)['after_arc_m'], .3)
        reference = tracker.build(guide, 0., path[0])
        self.assertIsNone(reference.next_corner)

    def test_single_vertex_bias_is_reduced_by_supported_fit(self):
        """较大单点偏差应显著减弱；此用例不宣称所有噪声和 NLOS 都能被拒绝。"""
        path = ((0., 0.), (.7, 0.), (1., .05), (1.08, -.02), (2., 0.))
        tracker, guide = TrailReferenceTracker(), self.guide(path)
        cosine, _ = tracker.supported_turn(guide, tracker.cumulative(path), 2)
        adjacent_cosine, _ = adjacent_turn(guide, tracker.cumulative(path), 2)
        self.assertLess(math.acos(cosine), math.acos(adjacent_cosine)*.6)

    def test_right_angle_has_two_real_supported_directions(self):
        """真实 90° 转弯在两侧有足够轨迹后仍得到确认，修复不能抹掉绕障方向。"""
        path = ((0., 0.), (1., 0.), (1., 1.))
        tracker, guide = TrailReferenceTracker(), self.guide(path)
        cosine, evidence = tracker.supported_turn(guide, tracker.cumulative(path), 1)
        self.assertAlmostEqual(cosine, 0.)
        self.assertAlmostEqual(dict(evidence)['angle_rad'], math.pi/2.)
        reference = tracker.build(guide, 0., (0., 0.))
        self.assertEqual(reference.next_corner, (1., 0.))
        self.assertGreater(reference.goal[1], 0.)

    def test_full_reversal_keeps_signed_direction(self):
        """有顺序的拟合必须保留 180° 折返；无符号主方向会把它错误当共线直行。"""
        path = ((0., 0.), (1., 0.), (0., 0.))
        tracker, guide = TrailReferenceTracker(), self.guide(path)
        cosine, evidence = tracker.supported_turn(guide, tracker.cumulative(path), 1)
        self.assertAlmostEqual(cosine, -1.)
        self.assertAlmostEqual(dict(evidence)['angle_rad'], math.pi)
        self.assertEqual(tracker.build(guide, 0., (0., 0.)).goal, (1., 0.))

    def test_prediction_cannot_supply_missing_real_corner_support(self):
        """已有真实转角只有短短 8 cm 时，长速度预测段不能替真实测量确认转角。"""
        path = ((0., 0.), (1., 0.), (1., .08), (1., 1.))
        tracker, guide = TrailReferenceTracker(), self.guide(path, prediction=True)
        self.assertIsNone(tracker.supported_turn(guide, tracker.cumulative(path), 1))
        # 实际转折点仍保留，后续有效历史够长时才形成角点任务。
        self.assertIn((1., 0.), guide.path)
        actual_guide = self.guide(path)
        cosine, _ = tracker.supported_turn(actual_guide, tracker.cumulative(path), 1)
        self.assertAlmostEqual(cosine, 0.)

    def test_corner_evidence_is_in_diagnostics(self):
        """验收可以看到转角来自多长的实际轨迹支持，不能只看到一个来源不明的角点。"""
        guide = self.guide(((0., 0.), (1., 0.), (1., 1.)))
        reference = TrailReferenceTracker().build(guide, 0., (0., 0.))
        evidence = reference.diagnostics()['corner_evidence']
        self.assertAlmostEqual(evidence['before_arc_m'], .3)
        self.assertAlmostEqual(evidence['after_arc_m'], .3)
        self.assertAlmostEqual(evidence['angle_rad'], math.pi/2.)

    def test_seeded_straight_uwb_noise_does_not_freeze_forward_reference(self):
        """连续 30 s 的 15 mm 测量噪声和 2 s 起步落后不能变成固定绕角停车点。"""
        for seed in (7, 91, 24329, 20261004, 1024):
            with self.subTest(seed=seed):
                rng = random.Random(seed)
                intent, tracker, old_tracker = FollowIntent(max_prediction=1.4), TrailReferenceTracker(), TrailReferenceTracker()
                old_tracker.supported_turn = adjacent_turn
                old_corners, new_corners, longest_hold, held = 0, 0, 0, 0
                previous = None
                for index in range(601):
                    elapsed = index*.05
                    robot = (.5*max(0., elapsed-2.), 0.)
                    measured = (2.+.5*elapsed+rng.gauss(0., .015), rng.gauss(0., .015))
                    self.assertTrue(intent.update(measured, 1.+elapsed, robot_position=robot, velocity=(.5, 0.)))
                    guide = intent.guide(1.+elapsed, 1.2)
                    reference = tracker.build(guide, 1.95, robot)
                    old = old_tracker.build(guide, 1.95, robot)
                    self.assertEqual(reference.status, 'FRESH')
                    if index > 100:
                        old_corners += old.next_corner is not None
                        new_corners += reference.next_corner is not None
                        self.assertGreater(reference.goal[0]-robot[0], 1.55)
                        held = held+1 if previous is not None and math.dist(previous, reference.goal) < 1e-6 else 0
                        longest_hold = max(longest_hold, held)
                    previous = reference.goal
                # 先证明输入确实使旧判定出错，再检查新方案；这不是仅断言辅助函数返回值。
                self.assertGreater(old_corners, 100)
                self.assertEqual(new_corners, 0)
                self.assertLess(longest_hold*.05, .3)


if __name__ == '__main__':
    unittest.main()
