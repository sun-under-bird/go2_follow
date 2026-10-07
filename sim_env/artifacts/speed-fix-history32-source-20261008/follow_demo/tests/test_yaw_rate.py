"""验证角速度实验控制的状态边界；逻辑通过不能代替真实步态基准。"""
import math
import unittest
from follow_demo.yaw_rate import YawRateServo


class YawRateServoTests(unittest.TestCase):
    """覆盖限幅、饱和、换向和复位，避免以隐藏旧方向或最低转速换取通过。"""

    def test_no_minimum_turn_is_added(self):
        """小角速度保持连续且趋近零，不存在最低速度抬升。"""
        servo = YawRateServo()
        small = servo.update(.5, 1e-5, 0.)
        self.assertGreater(small, 0.)
        self.assertLess(small, 1e-4)
        servo.reset()
        self.assertEqual(servo.update(.5, 0., 0.), 0.)

    def test_saturation_does_not_accumulate_unreachable_integral(self):
        """持续请求超能力转向时输入限幅，积分不向饱和方向增长。"""
        servo = YawRateServo()
        for _ in range(500):
            self.assertEqual(servo.update(0., 1., 0.), 1.)
        self.assertEqual(servo.integral, 0.)

    def test_reversal_does_not_keep_old_heading_sign(self):
        """在原地正转稳态换向，前馈和反馈当拍改变模型输入方向。"""
        servo = YawRateServo()
        for _ in range(200):
            servo.update(0., .4, .4)
        self.assertLess(servo.update(0., -.4, .4), 0.)

    def test_reset_removes_old_feedback_state(self):
        """暂停或重置后清积分与测量滤波，下一段不会复用上一转向状态。"""
        servo = YawRateServo()
        servo.update(.5, .2, 0.)
        self.assertNotEqual(servo.integral, 0.)
        servo.reset()
        self.assertEqual(servo.integral, 0.)
        self.assertIsNone(servo.filtered_rate)
        self.assertEqual(servo.last['output'], 0.)

    def test_reverse_error_can_release_an_existing_integral(self):
        """输出与误差方向相反时允许积分回落，而不是永久冻结饱和记忆。"""
        servo = YawRateServo(kp=0.)
        servo.integral = 1.
        before = servo.integral
        servo.update(.5, 0., .2)
        self.assertLess(servo.integral, before)

    def test_invalid_sensor_value_cannot_produce_torque_command(self):
        """非有限测量必须停止实验状态并报错，不能经限幅变成有效输入。"""
        servo = YawRateServo()
        with self.assertRaises(ValueError):
            servo.update(.5, .3, math.nan)
        self.assertEqual(servo.integral, 0.)


if __name__ == '__main__':
    unittest.main()
