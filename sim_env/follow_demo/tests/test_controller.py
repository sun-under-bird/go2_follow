"""验证测量时间对齐和关键停车语义，避免仅测试实现细节。"""
import math
import unittest
from follow_demo.controller import FollowController, Pose, PoseHistory


class ControllerTests(unittest.TestCase):
    """覆盖移动机器人下的延迟 UWB、异常输入和指令限制。"""
    def test_delayed_measurement_uses_capture_pose(self):
        """机器人运动时，同一静止目标不应因传输延迟被估计成移动目标。"""
        controller = FollowController()
        for index in range(101):
            controller.history.add(Pose(index * 0.01, index * 0.01, 0, 0))
        self.assertTrue(controller.observe(4.5, 0, 0.5))
        self.assertAlmostEqual(controller.target[0], 5.0)

    def test_heading_interpolation_crosses_pi(self):
        """跨 ±π 插值应沿短弧，不应错误地转到零度。"""
        history = PoseHistory()
        history.add(Pose(0, 0, 0, math.radians(179)))
        history.add(Pose(1, 0, 0, math.radians(-179)))
        self.assertAlmostEqual(abs(history.at(0.5).yaw), math.pi)

    def test_input_loss_clears_previous_command(self):
        """在运动中断开目标数据，下一次输出必须为零，恢复不能残留旧加速度。"""
        controller = FollowController()
        for index in range(100):
            now = index * 0.02
            controller.history.add(Pose(now, 0, 0, 0))
            controller.observe(4, 0, now)
            command = controller.step(now)
        self.assertGreater(command[0], 0.3)
        self.assertEqual(controller.step(2.0, signal_valid=False), (0.0, 0.0))
        self.assertEqual(controller.acceleration, [0.0, 0.0])
        self.assertEqual(controller.state, 'TARGET_LOST')

    def test_stale_data_cannot_keep_robot_moving(self):
        """有最新里程计但没有新目标时，历史目标不能无限外推。"""
        controller = FollowController()
        controller.history.add(Pose(0, 0, 0, 0))
        controller.observe(5, 0, 0)
        controller.history.add(Pose(1, 0, 0, 0))
        self.assertEqual(controller.step(1), (0.0, 0.0))
        self.assertEqual(controller.state, 'TARGET_LOST')

    def test_bad_measurement_does_not_refresh_validity(self):
        """NaN 与不合理跳点不能覆盖有效目标或刷新数据年龄。"""
        controller = FollowController()
        controller.history.add(Pose(0, 0, 0, 0))
        controller.observe(3, 0, 0)
        controller.history.add(Pose(0.1, 0, 0, 0))
        self.assertFalse(controller.observe(float('nan'), 0, 0.1))
        self.assertFalse(controller.observe(30, 0, 0.1))
        self.assertEqual(controller.target_stamp, 0)

    def test_nominal_acceleration_and_velocity_are_bounded(self):
        """普通起步不能突破速度或加速度限制，不把低速提升到最低步行速度。"""
        controller = FollowController()
        previous = 0.0
        for index in range(100):
            now = index * 0.02
            controller.history.add(Pose(now, 0, 0, 0))
            controller.observe(8, 0, now)
            velocity, angular = controller.step(now)
            self.assertLessEqual(velocity, controller.max_speed)
            self.assertLessEqual(abs(velocity - previous), 0.45 * 0.02 + 1e-9)
            previous = velocity

    def test_smoothing_cannot_exceed_output_limit_when_target_slows(self):
        """接近速度上限时目标停下，残余正加速度也不能突破输出硬限制。"""
        controller = FollowController()
        controller.history.add(Pose(1, 0, 0, 0))
        controller.observe(1.9, 0, 1)
        controller.command = [0.479, 0.0]
        controller.acceleration = [0.45, 0.0]
        velocity, _ = controller.step(1)
        self.assertGreaterEqual(velocity, 0.0)
        self.assertLessEqual(velocity, controller.max_speed)


if __name__ == '__main__':
    unittest.main()
