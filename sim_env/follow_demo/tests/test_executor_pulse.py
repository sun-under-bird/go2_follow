"""检查独立唤醒的生命周期、失败可见性与验收身份，不启动仿真。"""
import threading
import time
import unittest
from unittest.mock import Mock

from follow_demo.executor_pulse import ExecutorPulse
from follow_demo.tests.check_navigation_demo import verify_executor_wake_mode


class ExecutorPulseTests(unittest.TestCase):
    """覆盖关闭竞争及身份错配，防止待销毁执行器被后台线程继续访问。"""

    def test_close_stops_wakes_for_both_executors(self):
        """两个等待者均收到 guard 唤醒；close 返回后不再触碰执行器。"""
        seen = threading.Event()
        first, second = Mock(), Mock()
        second.wake.side_effect = seen.set
        pulse = ExecutorPulse((first, second), period=.005)
        pulse.start()
        try:
            self.assertTrue(seen.wait(1))
        finally:
            pulse.close()
        count = first.wake.call_count
        self.assertGreaterEqual(count, 1)
        self.assertEqual(count, second.wake.call_count)
        time.sleep(.02)
        self.assertEqual(count, first.wake.call_count)
        self.assertFalse(pulse.diagnostics()['worker_alive'])
        self.assertEqual('', pulse.diagnostics()['error'])

    def test_failure_visible_and_unstarted_close_safe(self):
        """guard 条件失效时退出且报告异常；native 对照未启动的线程也能关闭。"""
        executor = Mock()
        executor.wake.side_effect = RuntimeError('已销毁的执行器')
        pulse = ExecutorPulse((executor,), period=.005)
        pulse.start()
        pulse.thread.join(1)
        pulse.close()
        self.assertEqual('已销毁的执行器', pulse.diagnostics()['error'])
        ExecutorPulse(()).close()

    def test_acceptance_requires_stable_explicit_identity(self):
        """同源码下原生等待和单调唤醒必须分开验收；旧服务不能被默认为修复版。"""
        self.assertEqual('steady', verify_executor_wake_mode([dict(executor_wake_mode='steady')], 'steady'))
        for rows, expected in [([], None), ([{}], None), ([dict(executor_wake_mode='native')], 'steady'),
                               ([dict(executor_wake_mode='steady'), dict(executor_wake_mode='native')], None)]:
            with self.subTest(rows=rows), self.assertRaises(RuntimeError):
                verify_executor_wake_mode(rows, expected)


if __name__ == '__main__':
    unittest.main()
