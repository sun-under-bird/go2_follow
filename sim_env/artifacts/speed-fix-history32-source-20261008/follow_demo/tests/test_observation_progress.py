"""覆盖移动中已经取得相关信息的观察任务，防止信息收益下降造成多余停转。"""
import unittest
from follow_demo.controller import Pose
from follow_demo.local_map import RollingMap
from follow_demo.local_planner import Plan
from follow_demo.observation import ObservationSession
from follow_demo.observation_geometry import observation_view, check_observation_position


class ObservationProgressTests(unittest.TestCase):
    """用采集时间明确的相关前沿，区分信息完成、实际转向与新可通行通道。"""
    def setUp(self):
        """构造向侧前方取景的任务，当前朝向尚未对准且只确认起始净空。"""
        self.grid = RollingMap(static_history=True)
        self.grid.confirm_start(0.,0.,1.)
        self.grid.last_depth,self.grid.frames = 1.,1
        free,_,_ = self.grid.layers(1.)
        expected = observation_view(self.grid,free,[.06,0.],1.)['unknown_cells']
        region = [self.grid.point(self.grid.cell(0.,0.))]
        self.plan = Plan('OBSERVING','GOAL_UNOBSERVED',[[0.,0.],[.06,0.]],1.,
                         observation_region=region,observation_cells=[list(cell) for cell in expected])
        self.session = ObservationSession()
        self.session.begin(self.plan,Pose(1.,0.,0.,0.),1.,self.grid)

    def reveal(self, cells, stamp, occupied=False):
        """仅写入指定世界格的模拟测量，保留障碍与自由状态的不同含义。"""
        for col,row in cells:
            cell = self.grid.cell((col+.5)*self.grid.resolution,(row+.5)*self.grid.resolution)
            self.grid.seen[cell],self.grid.occupied[cell] = stamp,occupied

    def new_frame(self, stamp):
        """模拟任务开始后确实接入一帧深度，不能仅靠帧计数制造相关收益。"""
        self.grid.last_depth,self.grid.frames = stamp,2

    def test_revealed_frontier_finishes_before_low_gain_rejects_pose(self):
        """剩余收益低是已获得信息的结果时，移动阶段直接完成信息任务，无需再停转。"""
        self.reveal(self.session.expected_cells,1.2)
        self.new_frame(1.2)
        check = check_observation_position(self.grid,[0.,0.],1.,self.session.expected_cells,1.3)
        self.assertEqual(check['reason'],'OBSERVATION_LOW_GAIN')
        self.assertEqual(self.session.advance(Pose(1.3,0.,0.,0.),1.3,self.grid),'OBSERVATION_COMPLETE')
        self.assertEqual(self.session.last_outcome['stage'],'MOVE')
        self.assertEqual(self.session.last_outcome['evidence_since'],1.)
        self.assertIsNone(self.session.admitted_position)
        self.assertGreater(self.session.relevant_free_gain,0)

    def test_info_acquired_before_region_does_not_require_waypoint_arrival(self):
        """途中揭示原相关前沿可以取消取景任务，但不会把未到达的参考点伪报成到达。"""
        self.reveal(self.session.expected_cells,1.2)
        self.new_frame(1.2)
        actual = Pose(1.3,.4,0.,0.)
        self.assertFalse(self.session.inside_region(actual,self.grid))
        self.assertEqual(self.session.advance(actual,1.3,self.grid),'OBSERVATION_COMPLETE')
        self.assertIsNone(self.session.admitted_position)
        self.assertEqual(self.session.last_outcome['position'],[.4,0.])

    def test_new_wall_completes_information_without_creating_corridor(self):
        """相关新障碍也能消除多余观察，但新增自由格与可通行格必须仍为零。"""
        farthest = sorted(self.session.expected_cells,key=lambda cell: cell[0]**2+cell[1]**2,reverse=True)
        self.reveal(farthest[:self.session.minimum_relevant_gain],1.2,occupied=True)
        self.new_frame(1.2)
        self.assertEqual(self.session.advance(Pose(1.3,0.,0.,0.),1.3,self.grid),'OBSERVATION_COMPLETE')
        self.assertEqual(self.session.relevant_free_gain,0)
        self.assertEqual(self.session.relevant_allowed_gain,0)
        self.assertEqual(self.session.last_outcome['relevant_allowed_gain'],0)

    def test_pre_task_evidence_cannot_complete_move(self):
        """任务开始前采到的相关地图不能凭接入时间较晚而算成移动中的新信息。"""
        self.reveal(self.session.expected_cells,.9)
        self.new_frame(1.2)
        self.assertNotEqual(self.session.advance(Pose(1.3,0.,0.,0.),1.3,self.grid),'OBSERVATION_COMPLETE')
        self.assertEqual(self.session.relevant_fresh_gain,0)

    def test_future_obstacle_timestamp_cannot_complete_move(self):
        """持久占用标记不能绕过采集时间上界，未来时间的障碍证据不得计为当前收益。"""
        farthest = sorted(self.session.expected_cells,key=lambda cell: cell[0]**2+cell[1]**2,reverse=True)
        self.reveal(farthest[:self.session.minimum_relevant_gain],3.,occupied=True)
        self.new_frame(1.2)
        self.assertNotEqual(self.session.advance(Pose(1.3,0.,0.,0.),1.3,self.grid),'OBSERVATION_COMPLETE')
        self.assertEqual(self.session.relevant_fresh_gain,0)

    def test_new_frames_and_unrelated_cells_cannot_complete_move(self):
        """其他方向的地图增长加新帧仍不构成原相关前沿的证据。"""
        self.grid.seen[self.grid.cell(-3.,-3.)] = 1.2
        self.new_frame(1.2)
        self.assertIsNone(self.session.advance(Pose(1.3,.4,0.,0.),1.3,self.grid))
        self.assertEqual(self.session.stage,'MOVE')
        self.assertEqual(self.session.relevant_fresh_gain,0)

    def test_related_cell_changes_without_new_depth_frame_do_not_complete(self):
        """世界格状态变化还须有任务开始后的真实新深度，不能靠地图维护完成观察。"""
        self.reveal(self.session.expected_cells,1.2)
        self.assertNotEqual(self.session.advance(Pose(1.3,0.,0.,0.),1.3,self.grid),'OBSERVATION_COMPLETE')


if __name__ == '__main__':
    unittest.main()
