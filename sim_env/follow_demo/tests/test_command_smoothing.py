"""复现反向目标变化导致命令继续增大的根因，并验证限幅和正常追踪。"""
import unittest
from follow_demo.command_smoothing import smooth_axis


class CommandSmoothingTests(unittest.TestCase):
    """线速度和正负转向都不能因残余加速度朝目标反方向变化。"""
    def test_target_change_cancels_opposite_acceleration(self):
        """向下减速、反向转弯、负转速减幅须先撤销旧加速度。"""
        for previous,acceleration,desired in ((.65,.7,.45),(.6,1.,-.2),(-.6,-1.,-.3),(.7,.4,.5)):
            actual,after,reset=smooth_axis(previous,acceleration,desired,.02,1.6,4.,-1.,1.)
            self.assertTrue(reset)
            self.assertLess(abs(actual-desired),abs(previous-desired))
            self.assertLessEqual(abs(after),.08+1e-12)

    def test_steady_target_retains_normal_jerk_and_acceleration_bounds(self):
        """常规加速不重置状态，不能借修复目标反转而直接跳到高速。"""
        command=acceleration=0.
        for _ in range(200):
            previous,old=command,acceleration
            command,acceleration,reset=smooth_axis(command,acceleration,.8,.02,.6,2.,0.,.8)
            self.assertFalse(reset)
            self.assertGreaterEqual(command,previous)
            self.assertLessEqual(command,.8)
            self.assertLessEqual(abs(command-previous),.6*.02+1e-12)
            if command<.8:
                self.assertLessEqual(abs(acceleration-old),2.*.02+1e-12)
        self.assertEqual(command,.8)

    def test_phase_cap_overrides_comfort_state(self):
        """突然收紧阶段上限仍立即限幅，不能等待jerk慢慢降下来。"""
        command,acceleration,_=smooth_axis(.8,.5,.8,.02,.6,2.,0.,.4)
        self.assertEqual((command,acceleration),(.4,0.))

    def test_invalid_input_rejected(self):
        """拒绝不可执行的时间或非有限输入。"""
        for dt in (0.,-1.,float('nan')):
            with self.assertRaises(ValueError):
                smooth_axis(0.,0.,.8,dt,.6,2.,0.,.8)


if __name__=='__main__':
    unittest.main()
