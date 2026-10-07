"""反例：观察到少量新格但没有完整通道时，不能取消仍有效的接近任务。"""
import unittest
from follow_demo.controller import Pose
from follow_demo.tests import test_observation_progress as helper_tests


class CameraObservationProgressTests(unittest.TestCase):
    """直接复现真实地图中的零通道收益，保留相机模式的移动上下文。"""

    def context(self):
        """复用一个待揭示前沿，改用真实相机任务语义后重新建立快照。"""
        helper = helper_tests.ObservationProgressTests()
        helper.setUp()
        helper.grid.camera_observation = True
        helper.session.begin(helper.plan, Pose(1., 0., 0., 0.), 1., helper.grid)
        return helper

    def test_isolated_new_free_cells_keep_move(self):
        """远端零散自由证据不能形成机身通道，也不能触发清空路线。"""
        helper = self.context()
        far = sorted(helper.session.expected_cells, key=lambda p: p[0]**2+p[1]**2, reverse=True)
        helper.reveal(far[:15], 1.2)
        helper.new_frame(1.2)
        self.assertIsNone(helper.session.advance(Pose(1.3, .4, 0., 0.), 1.3, helper.grid))
        self.assertEqual(helper.session.stage, 'MOVE')
        self.assertEqual(helper.session.relevant_allowed_gain, 0)
        self.assertGreater(helper.session.relevant_free_gain, 0)
        self.assertIsNotNone(helper.session.plan)

    def test_generic_information_does_not_finish_move(self):
        """即使整片前沿变为已知，实际可达路线仍应由后台搜索接替而非信息计数宣告。"""
        helper = self.context()
        helper.reveal(helper.session.expected_cells, 1.2)
        helper.new_frame(1.2)
        self.assertIsNone(helper.session.advance(Pose(1.3, .4, 0., 0.), 1.3, helper.grid))
        self.assertEqual(helper.session.stage, 'MOVE')
        self.assertIsNone(helper.session.last_outcome)


if __name__ == '__main__':
    unittest.main()
