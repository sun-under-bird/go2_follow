"""重复关闭 API 请求不能在资源回收途中再次发送 SIGINT。"""
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from follow_demo.app import Runtime


class ShutdownTests(unittest.TestCase):
    """不启动仿真或发信号，仅检查真实退出入口的幂等性。"""

    def test_repeated_shutdown_starts_one_signal_timer(self):
        """检查器和停止脚本先后请求关闭时，只安排一次退出且先下急停。"""
        runtime = SimpleNamespace(shutdown_lock=threading.Lock(), shutdown_requested=False,
                                  simulation=SimpleNamespace(action=Mock()))
        with patch('follow_demo.app.threading.Timer') as timer:
            self.assertTrue(Runtime.request_shutdown(runtime))
            self.assertFalse(Runtime.request_shutdown(runtime))
            self.assertFalse(Runtime.request_shutdown(runtime))
            runtime.simulation.action.assert_called_once_with('estop')
            timer.assert_called_once()
            timer.return_value.start.assert_called_once()


if __name__ == '__main__':
    unittest.main()
