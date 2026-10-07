"""独立检查人的轨迹意图；这些测试不制造地图自由证据，也不启动 ROS 或仿真。"""
import math
import unittest

from follow_demo.follow_intent import FollowIntent


class FollowIntentTests(unittest.TestCase):
    """通过折返、失联和有限历史反例验证弧长意图的输入与几何边界。"""

    def assert_point_close(self, actual, expected, places=8):
        """逐坐标比较几何插值结果，避免把浮点误差当作路径变化。"""
        self.assertEqual(len(actual), 2)
        for value, wanted in zip(actual, expected):
            self.assertAlmostEqual(value, wanted, places=places)

    def test_right_angle_uses_walked_arc_instead_of_diagonal(self):
        """退后 1.5 m 必须经过真实拐点，不能把人的直角抹成斜线。"""
        intent = FollowIntent()
        for stamp, point in enumerate(((0., 0.), (1., 0.), (1., 1.))):
            self.assertTrue(intent.update(point, stamp * .1, velocity=(0., 0.)))
        hint = intent.hint(.2, 1.5)
        self.assert_point_close(hint.follow_point, (.5, 0.))
        self.assertEqual(hint.path[1:], ((1., 0.), (1., 1.)))
        self.assertAlmostEqual(hint.observed_length, 2.)
        self.assertTrue(hint.history_complete)

    def test_same_line_full_reversal_keeps_turnaround(self):
        """端点回到初始位置时仍须保留实际走过的 2 m，不能合并成零长度。"""
        intent = FollowIntent()
        for stamp, point in enumerate(((0., 0.), (1., 0.), (0., 0.))):
            intent.update(point, stamp * .1, velocity=(0., 0.))
        self.assertEqual(intent.observed_path(), ((0., 0.), (1., 0.), (0., 0.)))
        hint = intent.hint(.2, 1.5)
        self.assert_point_close(hint.follow_point, (.5, 0.))
        self.assertEqual(hint.path[1:], ((1., 0.), (0., 0.)))
        self.assertAlmostEqual(hint.observed_length, 2.)

    def test_consecutive_turn_keeps_prior_upper_passage(self):
        """人在第二个拐点向右走时，跟随意图仍沿上一段向下路线后退。"""
        intent = FollowIntent()
        trail = ((2., 0.), (2., 1.1), (4., 1.5), (4., -.5), (4.1, -.5))
        for stamp, point in enumerate(trail):
            velocity = (.5, 0.) if stamp == len(trail)-1 else (0., 0.)
            intent.update(point, stamp * .1, velocity=velocity)
        hint = intent.hint(.4, 2., prediction_horizon=.3)
        # 预测终点为 (4.25,-.5)，先退 0.25 m，再沿向下段退 1.75 m。
        self.assert_point_close(hint.follow_point, (4., 1.25))
        self.assertEqual(hint.path[1:], ((4., -.5), (4.1, -.5), (4.25, -.5)))
        self.assertEqual(hint.source, 'observed_target_trail')
        self.assertFalse(hint.virtual_stem_used)

    def test_straight_sampling_merges_without_changing_arc(self):
        """持续直行只保留必要端点，不能因降采样损失前进弧长。"""
        intent = FollowIntent()
        for index in range(31):
            intent.update((index*.1, 0.), index*.05, velocity=(0., 0.))
        self.assertLessEqual(len(intent.observed_path()), 3)
        self.assertAlmostEqual(intent.diagnostics()['observed_length'], 3.)
        self.assert_point_close(intent.hint(1.5, 1.).follow_point, (2., 0.))

    def test_subsample_latest_endpoint_remains_visible(self):
        """最新点不足降采样间距也用于当前意图，不能冻结到上一采样点。"""
        intent = FollowIntent()
        intent.update((0., 0.), 1., velocity=(0., 0.))
        intent.update((.03, 0.), 1.1, velocity=(0., 0.))
        self.assertEqual(intent.observed_path(), ((0., 0.), (.03, 0.)))
        self.assert_point_close(intent.hint(1.1, 0.).follow_point, (.03, 0.))

    def test_virtual_stem_uses_initial_robot_position_only(self):
        """短历史使用固定初始意图线段，机器人之后绕行不能重画这段历史。"""
        intent = FollowIntent()
        intent.update((2., 0.), 1., robot_position=(0., 0.), velocity=(0., 0.))
        intent.update((2.1, 0.), 1.1, robot_position=(0., 1.), velocity=(0., 0.))
        hint = intent.hint(1.1, 1.8)
        self.assert_point_close(hint.follow_point, (.3, 0.))
        self.assertTrue(hint.virtual_stem_used)
        self.assertFalse(hint.history_complete)
        self.assertEqual(hint.source, 'robot_virtual_stem')
        self.assertAlmostEqual(hint.observed_length, .1)

    def test_sufficient_real_history_no_longer_uses_virtual_stem(self):
        """实际历史够长后跟随点只落在人的轨迹上，虚拟起始线段退出当前提示。"""
        intent = FollowIntent()
        intent.update((2., 0.), 1., robot_position=(0., 0.), velocity=(0., 0.))
        intent.update((2., 2.), 1.1, velocity=(0., 0.))
        hint = intent.hint(1.1, 1.5)
        self.assert_point_close(hint.follow_point, (2., .5))
        self.assertTrue(hint.history_complete)
        self.assertFalse(hint.virtual_stem_used)
        self.assertEqual(hint.path, ((2., .5), (2., 2.)))

    def test_insufficient_history_stops_at_oldest_available_point(self):
        """没有虚拟起始点时不得编造更早轨迹，退后距离不足会明确显示历史不完整。"""
        intent = FollowIntent()
        intent.update((3., 2.), 1., velocity=(0., 0.))
        hint = intent.hint(1., 2.)
        self.assertEqual(hint.follow_point, (3., 2.))
        self.assertEqual(hint.path, ((3., 2.),))
        self.assertFalse(hint.history_complete)
        self.assertFalse(hint.virtual_stem_used)

    def test_stale_and_future_queries_reject_history(self):
        """过期目标和未来采样均不能用于规划意图；合法新鲜边界仍可读取。"""
        intent = FollowIntent()
        intent.update((2., 0.), 1., velocity=(0., 0.))
        self.assertIsNotNone(intent.hint(1.44, 1.))
        self.assertIsNone(intent.hint(1.46, 1.))
        self.assertIsNone(intent.hint(.99, 1.))

    def test_long_loss_does_not_connect_unknown_target_motion(self):
        """失联超过有效期后重建，不能连起旧人轨迹与新 UWB 点。"""
        intent = FollowIntent()
        intent.update((2., 0.), 1., robot_position=(0., 0.), velocity=(.5, 0.))
        intent.update((2.1, 0.), 1.1, velocity=(.5, 0.))
        intent.update((4., 2.), 2., robot_position=(3., 2.), velocity=(0., 0.))
        self.assertEqual(intent.observed_path(), ((4., 2.),))
        hint = intent.hint(2., .5)
        self.assert_point_close(hint.follow_point, (3.5, 2.))
        self.assertEqual(hint.target_stamp, 2.)
        self.assertAlmostEqual(hint.observed_length, 0.)

    def test_duplicate_and_out_of_order_packets_do_not_rewrite_latest(self):
        """重复和乱序 UWB 不覆盖最新时间与折线，正常意图仍保持有效。"""
        intent = FollowIntent()
        intent.update((2., 0.), 1., velocity=(0., 0.))
        intent.update((2., 1.), 1.1, velocity=(0., 0.))
        before = intent.observed_path()
        self.assertFalse(intent.update((99., 99.), 1.1, velocity=(0., 0.)))
        self.assertFalse(intent.update((99., 99.), 1., velocity=(0., 0.)))
        self.assertEqual(intent.observed_path(), before)
        hint = intent.hint(1.1, .5)
        self.assertEqual(hint.latest_target, (2., 1.))
        self.assertEqual(hint.target_stamp, 1.1)

    def test_signal_invalidates_without_inventing_a_new_stamp(self):
        """显式信号失效立即取消提示，下一真实新帧才重新有效。"""
        intent = FollowIntent()
        intent.update((2., 0.), 1., velocity=(0., 0.))
        self.assertFalse(intent.update((999., 999.), 1.1, valid=False))
        self.assertIsNone(intent.hint(1.1, 1.))
        self.assertEqual(intent.diagnostics()['target_stamp'], 1.)
        self.assertTrue(intent.update((2., .1), 1.2, velocity=(0., 0.)))
        self.assertIsNotNone(intent.hint(1.2, 1.))

    def test_invalid_coordinates_velocity_and_stamp_cancel_hint(self):
        """非有限坐标、错误维度、非有限速度与时间不进入意图历史。"""
        bad_cases = (((math.nan, 0.), 1.1, None), ((1.,), 1.1, None),
                     ((1., 2.), math.inf, None), ((1., 2.), '1.1', None),
                     ((1., 2.), 1.1, (math.inf, 0.)))
        for point, stamp, velocity in bad_cases:
            with self.subTest(point=point, stamp=stamp, velocity=velocity):
                intent = FollowIntent()
                intent.update((2., 0.), 1., velocity=(0., 0.))
                self.assertFalse(intent.update(point, stamp, velocity=velocity))
                self.assertEqual(intent.observed_path(), ((2., 0.),))
                self.assertIsNone(intent.hint(1.1, 1.))

    def test_prediction_is_bounded_by_speed_and_total_seconds(self):
        """预测包含采样年龄但有统一上限，不能因追赶需求无限外推。"""
        intent = FollowIntent(max_speed=1.5, max_prediction=.5)
        intent.update((2., 0.), 1., velocity=(100., 0.))
        hint = intent.hint(1.2, 0., prediction_horizon=10.)
        self.assertAlmostEqual(hint.prediction_seconds, .5)
        self.assert_point_close(hint.predicted_target, (2.75, 0.))
        self.assertEqual(hint.source, 'short_velocity_prediction')

    def test_stationary_motion_has_no_residual_prediction(self):
        """静止阈值内抖动不会生成预测段，避免停人时仍持续追逐虚拟目标。"""
        intent = FollowIntent()
        intent.update((2., 0.), 1., velocity=(.02, .01))
        hint = intent.hint(1.2, 0.)
        self.assertAlmostEqual(hint.prediction_seconds, 0.)
        self.assertEqual(hint.predicted_target, (2., 0.))
        self.assertEqual(hint.source, 'observed_target_trail')

    def test_estimated_velocity_is_world_difference_when_not_supplied(self):
        """未提供速度时使用世界目标差分；初帧不会产生非零虚构速度。"""
        intent = FollowIntent()
        intent.update((2., 0.), 1.)
        self.assertEqual(intent.hint(1., 0.).predicted_target, (2., 0.))
        intent.update((2.1, 0.), 1.1)
        hint = intent.hint(1.1, 0., prediction_horizon=.3)
        # dt=0.1，低通 alpha=0.25，差分速度=1 m/s，预测 0.075 m。
        self.assert_point_close(hint.predicted_target, (2.175, 0.))

    def test_length_limit_interpolates_head_without_exceeding_budget(self):
        """长轨迹截掉旧头部并保留最新端点；不增加未经测量的方向。"""
        intent = FollowIntent(max_history_length=1.)
        intent.update((0., 0.), 1., robot_position=(-2., 0.), velocity=(0., 0.))
        intent.update((2., 0.), 1.1, velocity=(0., 0.))
        self.assertEqual(intent.observed_path(), ((1., 0.), (2., 0.)))
        self.assertAlmostEqual(intent.diagnostics()['observed_length'], 1.)
        self.assertFalse(intent.diagnostics()['virtual_stem_available'])

    def test_point_limit_retains_tail_turns_and_latest_endpoint(self):
        """点数不足时只丢旧头部，尾部折返不可被简化来凑上限。"""
        intent = FollowIntent(max_points=6, max_history_length=100.)
        points = tuple((float(index % 2), 0.) for index in range(20))
        for index, point in enumerate(points):
            intent.update(point, index*.1, velocity=(0., 0.))
        observed = intent.observed_path()
        self.assertLessEqual(len(observed), 6)
        self.assertEqual(observed[-4:], points[-4:])
        self.assertEqual(observed[-1], points[-1])

    def test_reset_allows_new_time_origin_and_clears_previous_corner(self):
        """定位或场景重置后可从更早时间重新接入，上一轮拐点不得泄漏。"""
        intent = FollowIntent()
        intent.update((2., 0.), 10., velocity=(0., 0.))
        intent.update((2., 1.), 10.1, velocity=(0., 0.))
        intent.reset()
        self.assertIsNone(intent.hint(0., 1.))
        self.assertTrue(intent.update((5., 3.), 0., velocity=(0., 0.)))
        self.assertEqual(intent.observed_path(), ((5., 3.),))

    def test_inputs_are_copied_and_hint_has_no_map_claim(self):
        """调用方改列表不能污染历史；提示没有 free、allowed 或安全可执行标志。"""
        intent = FollowIntent()
        target, robot = [2., 0.], [0., 0.]
        intent.update(target, 1., robot_position=robot, velocity=(0., 0.))
        target[0], robot[1] = 99., 99.
        hint = intent.hint(1., 1.)
        self.assertEqual(hint.latest_target, (2., 0.))
        self.assert_point_close(hint.follow_point, (1., 0.))
        for name in ('free', 'allowed', 'safe', 'collision_free'):
            self.assertFalse(hasattr(hint, name))

    def test_bad_query_parameters_return_none(self):
        """负跟随距离、负预测和非有限查询参数不能输出误导提示。"""
        intent = FollowIntent()
        intent.update((2., 0.), 1., velocity=(0., 0.))
        for now, spacing, horizon in ((1., -1., .3), (1., 1., -1.),
                                     (math.nan, 1., .3), (1., math.inf, .3)):
            with self.subTest(now=now, spacing=spacing, horizon=horizon):
                self.assertIsNone(intent.hint(now, spacing, horizon))

    def test_invalid_limits_raise_value_error(self):
        """构造参数不允许零采样长度、过短历史和失效时间等退化值。"""
        for parameters in (dict(sample_distance=0.), dict(max_history_length=.01),
                           dict(max_points=3), dict(stale_timeout=0.),
                           dict(corner_angle=math.pi), dict(max_prediction=-1.)):
            with self.subTest(parameters=parameters):
                with self.assertRaises(ValueError):
                    FollowIntent(**parameters)


if __name__ == '__main__':
    unittest.main()
