"""检查最终仿真命令接收端，不允许隐藏的旧0.75上限截断已设置的0.8。"""
import threading
import unittest
import numpy as np
from follow_demo.simulation import Simulation


class SimulationCommandLimitTests(unittest.TestCase):
    """仅测试接收边界，不启动MuJoCo、ROS或后台服务。"""
    def receiver(self):
        """构造接收端最小状态，省去与本测试无关的物理模型加载。"""
        receiver = Simulation.__new__(Simulation)
        receiver.lock = threading.Lock()
        return receiver

    def test_requested_maximum_is_preserved_at_final_receiver(self):
        """0.8与±1.0完整进入策略，超过边界才限幅。"""
        receiver = self.receiver()
        receiver.set_command(.8,0,1.)
        np.testing.assert_array_equal(receiver.command,[.8,0,1.])
        receiver.set_command(1.2,0,-2.)
        np.testing.assert_array_equal(receiver.command,[.8,0,-1.])

    def test_invalid_command_stops(self):
        """无效数值不能绕过接收端限幅后进入运动策略。"""
        receiver = self.receiver()
        receiver.set_command(float('nan'),0,0)
        np.testing.assert_array_equal(receiver.command,[0,0,0])


if __name__ == '__main__':
    unittest.main()
