"""验证闭环 A/B 的实验身份，防止把另一执行模式误记为通过结果。"""
import unittest

from follow_demo.tests.check_navigation_demo import (verify_execution_mode, verify_planning_mode,
                                                    verify_observation_mode, verify_search_execution_mode)


class ExecutionModeTests(unittest.TestCase):
    """核对同源码不同启动参数时必须保留和校验的模式边界。"""

    def test_records_actual_mode_without_expected_argument(self):
        """省略期望参数仍能得到实际模式，且支持三种独立实验。"""
        for mode in ('heading', 'velocity', 'rate'):
            with self.subTest(mode=mode):
                self.assertEqual(verify_execution_mode([{'execution_mode': mode}] * 3), mode)

    def test_rejects_expected_mode_mismatch(self):
        """错误启动参数不能生成另一组实验的有效证据。"""
        with self.assertRaisesRegex(RuntimeError, '执行模式不符'):
            verify_execution_mode([{'execution_mode': 'heading'}], 'velocity')

    def test_rejects_mixed_mode_samples(self):
        """中途换实例或混合采样必须判失败，而非只看最终模式。"""
        with self.assertRaisesRegex(RuntimeError, '发生变化'):
            verify_execution_mode([{'execution_mode': 'heading'}, {'execution_mode': 'velocity'}])

    def test_rejects_old_server_without_mode_identity(self):
        """旧服务缺少模式身份时不能按默认值猜测。"""
        for rows in ([], [{}], [{'execution_mode': 'other'}]):
            with self.subTest(rows=rows), self.assertRaises(RuntimeError):
                verify_execution_mode(rows)


class PlanningModeTests(unittest.TestCase):
    """规划语义也必须明确，不能将新目标生成器与旧距离环报告混在一起。"""

    def test_records_supported_mode(self):
        """两个合法规划方式均保留真实服务身份。"""
        for mode in ('annulus', 'trail'):
            self.assertEqual(verify_planning_mode([{'planning_mode': mode}] * 2, mode), mode)

    def test_rejects_mismatch(self):
        """启动旧模式却验收新模式时必须失败。"""
        with self.assertRaisesRegex(RuntimeError, '规划模式不符'):
            verify_planning_mode([{'planning_mode': 'annulus'}], 'trail')

    def test_rejects_missing_and_mixed(self):
        """缺少服务身份和跨实例混合数据都不能作为对照证据。"""
        for rows in ([], [{}], [{'planning_mode': 'invalid'}],
                     [{'planning_mode': 'trail'}, {'planning_mode': 'annulus'}]):
            with self.subTest(rows=rows), self.assertRaises(RuntimeError):
                verify_planning_mode(rows)


class SearchExecutionModeTests(unittest.TestCase):
    """搜索方式是性能对照条件，缺失或混用时不能承认验收结果。"""

    def test_records_supported_modes(self):
        """记录线程或进程的真实身份。"""
        for mode in ('thread', 'process'):
            self.assertEqual(verify_search_execution_mode([{'search_execution_mode': mode}], mode), mode)

    def test_rejects_missing_mismatched_and_mixed(self):
        """禁止旧服务、参数错配和中途换服务形成有效对照。"""
        for rows, expected in (([], None), ([{}], None), ([{'search_execution_mode': 'other'}], None),
                               ([{'search_execution_mode': 'thread'}], 'process'),
                               ([{'search_execution_mode': 'thread'}, {'search_execution_mode': 'process'}], None)):
            with self.subTest(rows=rows), self.assertRaises(RuntimeError):
                verify_search_execution_mode(rows, expected)


class ObservationModeTests(unittest.TestCase):
    """相机真实几何的结果必须区别于旧二维覆盖预测。"""

    def test_records_supported_modes(self):
        """记录本次选用的观察模型。"""
        for mode in ('cone', 'camera'):
            self.assertEqual(verify_observation_mode([{'observation_mode': mode}], mode), mode)

    def test_rejects_wrong_or_mixed_identity(self):
        """模型缺失、预期错配和混合数据均不能生成通过结论。"""
        for rows, expected in (([], None), ([{}], None), ([{'observation_mode': 'cone'}], 'camera'),
                               ([{'observation_mode': 'cone'}, {'observation_mode': 'camera'}], None)):
            with self.subTest(rows=rows), self.assertRaises(RuntimeError):
                verify_observation_mode(rows, expected)


if __name__ == '__main__':
    unittest.main()
