"""验证有效观察区域、实际朝向和相关新深度，防止放宽容差造成假观察。"""
import copy
import math
import unittest
from follow_demo.controller import Pose
from follow_demo.local_map import RollingMap
from follow_demo.local_planner import Plan
from follow_demo.observation import ObservationSession
from follow_demo.observation_geometry import observation_view


class ObservationTests(unittest.TestCase):
    """用受控的地图采集时间覆盖观察闭环，不依赖 ROS 或物理引擎。"""
    def setUp(self):
        """构造起始净空及需要转向一弧度的真实未知前沿。"""
        self.grid = RollingMap(static_history=True)
        self.grid.confirm_start(0,0,1)
        self.grid.last_depth,self.grid.frames = 1.,1
        self.plan = self.make_plan(.06,[(0,0),(.02,0)])
        self.session = ObservationSession()
        self.session.begin(self.plan,Pose(1,0,0,0),1,self.grid)

    def make_plan(self, end, region_points):
        """从参考视点获取预测未知格，区域每个位置仍由运行时重新验证。"""
        free,_,_ = self.grid.layers(1.)
        expected = observation_view(self.grid,free,[end,0.],1.)['unknown_cells']
        region = [self.grid.point(self.grid.cell(*point)) for point in region_points]
        return Plan('OBSERVING','GOAL_UNOBSERVED',[[0.,0.],[end,0.]],1.,
                    observation_region=region,observation_cells=[list(cell) for cell in sorted(expected)])

    def reveal_related(self, stamp, occupied=False):
        """模拟相关前沿实际被新帧揭示；障碍命中也携带采集时间。"""
        for col,row in self.session.expected_cells:
            cell = self.grid.cell((col+.5)*self.grid.resolution,(row+.5)*self.grid.resolution)
            self.grid.seen[cell] = stamp
            self.grid.occupied[cell] = occupied

    def align(self):
        """先通过实际区域检查，再以实际朝向进入评估。"""
        self.assertEqual(self.session.advance(Pose(1.1,0,0,0),1.1,self.grid),'OBSERVATION_ORIENT')
        self.assertEqual(self.session.advance(Pose(1.3,0,0,1),1.3,self.grid),'OBSERVATION_EVALUATE')

    def test_actual_pose_and_related_new_frames_are_required(self):
        """旧帧和空帧不能完成观察，相关未知格必须在朝向到位后获得真实证据。"""
        self.align()
        self.grid.frames,self.grid.last_depth = 5,1.2
        self.reveal_related(1.2)
        self.assertIsNone(self.session.advance(Pose(2,0,0,1),2,self.grid))
        self.grid.last_depth = 2.
        self.assertIsNone(self.session.advance(Pose(2.1,0,0,1),2.1,self.grid))
        self.assertEqual(self.session.relevant_fresh_gain,0)
        self.reveal_related(2.1)
        self.assertEqual(self.session.advance(Pose(2.2,0,0,1),2.2,self.grid),'OBSERVATION_COMPLETE')
        self.assertTrue(self.session.excluded([0.,0.],1.))
        self.assertFalse(self.session.excluded([.5,0.],1.))

    def test_no_turn_progress_fails_without_changing_velocity(self):
        """实际朝向长期不变必须失败，不通过最低角速度或无限重发强推。"""
        self.session.advance(Pose(1.1,0,0,0),1.1,self.grid)
        self.assertEqual(self.session.advance(Pose(5.2,0,0,0),5.2,self.grid),'OBSERVATION_TURN_STALLED')
        self.assertEqual(self.session.stage,'IDLE')
        self.assertTrue(self.session.excluded([0.,0.],1.))

    def test_collapse_does_not_mutate_search_plan(self):
        """实际观察位置锚定不能污染搜索线程的路线或区域。"""
        before = copy.deepcopy(self.plan)
        self.session.advance(Pose(1.1,.02,0,0),1.1,self.grid)
        self.assertEqual(self.plan,before)
        self.assertEqual(self.session.plan.path,[[.02,0.]])

    def test_region_allows_useful_actual_position_short_of_reference(self):
        """距参考点大于旧容差但实际视野有效时可以观察，无需追厘米级终点。"""
        plan = self.make_plan(.16,[(.05,0),(.16,0)])
        self.session.begin(plan,Pose(1,.05,0,0),1,self.grid)
        self.assertEqual(self.session.advance(Pose(1.1,.05,0,0),1.1,self.grid),'OBSERVATION_ORIENT')
        self.assertEqual(self.session.admitted_position,[.05,0.])
        self.assertTrue(self.session.region_check['valid'])

    def test_short_of_region_cannot_finish_move(self):
        """距终点很近但尚未进入验证区域时不能提前宣告到达。"""
        plan = self.make_plan(.28,[(.23,0),(.28,0)])
        self.session.begin(plan,Pose(1,.10,0,0),1,self.grid)
        self.assertIsNone(self.session.advance(Pose(1.1,.10,0,0),1.1,self.grid))
        self.assertEqual(self.session.stage,'MOVE')
        self.assertEqual(self.session.advance(Pose(1.2,.23,0,0),1.2,self.grid),'OBSERVATION_ORIENT')

    def test_stopping_space_is_required_for_region_entry(self):
        """几何位置有收益也必须先确认当前实测运动有停车空间。"""
        self.assertIsNone(self.session.advance(Pose(1.1,0,0,0),1.1,self.grid,parking_safe=False))
        self.assertEqual(self.session.stage,'MOVE')
        self.assertEqual(self.session.region_check['reason'],'OBSERVATION_STOPPING_SPACE')

    def test_leaving_region_restores_move_reference(self):
        """残余位移离开有效区域时恢复移动，不能在旧锚点状态下完成观察。"""
        self.align()
        self.assertEqual(self.session.advance(Pose(1.5,.30,0,1),1.5,self.grid),'OBSERVATION_REGION_LEFT')
        self.assertEqual(self.session.stage,'MOVE')
        self.assertEqual(self.session.plan.path,self.plan.path)

    def test_drifted_heading_cannot_complete_with_new_depth(self):
        """实际朝向偏离时重新看向，即使相关新信息已到达也不能假完成。"""
        self.align()
        self.grid.last_depth,self.grid.frames = 2.,5
        self.reveal_related(2.)
        self.assertEqual(self.session.advance(Pose(2,0,0,1.4),2,self.grid),'OBSERVATION_ORIENT')
        self.assertEqual(self.session.stage,'ORIENT')
        self.assertEqual(self.session.tried,[])

    def test_unrelated_new_map_and_empty_frames_fail(self):
        """全图新增格和新帧计数不能代替目标前沿信息。"""
        self.align()
        self.grid.seen[self.grid.cell(-3.,-3.)] = 2.
        self.grid.last_depth,self.grid.frames = 3.4,10
        self.assertEqual(self.session.advance(Pose(3.4,0,0,1),3.4,self.grid),'OBSERVATION_NO_RELEVANT_GAIN')
        self.assertEqual(self.session.relevant_fresh_gain,0)

    def test_new_wall_is_information_but_not_free_space(self):
        """新看到的障碍可完成取景，但不会被统计为自由或可行驶通道。"""
        self.align()
        # 只揭示远端墙表面，不能把整片墙后未知区域都伪造成同一帧的障碍命中。
        cells = sorted(self.session.expected_cells,key=lambda cell: cell[0]**2+cell[1]**2,reverse=True)
        for col,row in cells[:self.session.minimum_relevant_gain]:
            cell = self.grid.cell((col+.5)*self.grid.resolution,(row+.5)*self.grid.resolution)
            self.grid.seen[cell],self.grid.occupied[cell] = 2.,True
        self.grid.last_depth,self.grid.frames = 2.,5
        result = self.session.advance(Pose(2.1,0,0,1),2.1,self.grid)
        self.assertEqual(result,'OBSERVATION_COMPLETE')
        self.assertGreaterEqual(self.session.relevant_fresh_gain,self.session.minimum_relevant_gain)
        self.assertEqual(self.session.relevant_free_gain,0)
        self.assertEqual(self.session.relevant_allowed_gain,0)

    def test_new_near_wall_can_reduce_roi_without_leaving_region(self):
        """姿态未动，新墙遮住原预测前沿时应识别真实障碍收益，不能恢复厘米级移动。"""
        self.align()
        for col,row in self.session.expected_cells:
            point = [(col+.5)*self.grid.resolution,(row+.5)*self.grid.resolution]
            forward = point[0]*math.cos(1.)+point[1]*math.sin(1.)
            if 1.3 <= forward <= 1.6:
                cell = self.grid.cell(*point)
                self.grid.seen[cell],self.grid.occupied[cell] = 2.,True
        self.grid.last_depth,self.grid.frames = 2.,5
        self.assertEqual(self.session.advance(Pose(2.1,0,0,1),2.1,self.grid),'OBSERVATION_COMPLETE')
        self.assertLess(self.session.region_check['overlap_ratio'],.30)
        self.assertGreaterEqual(self.session.relevant_fresh_gain,self.session.minimum_relevant_gain)
        self.assertEqual(self.session.relevant_free_gain,0)

    def test_missing_region_never_uses_distance_fallback(self):
        """缺少区域和前沿元数据时闭锁，不能回退到旧距离完成逻辑。"""
        plan = Plan('OBSERVING','GOAL_UNOBSERVED',[[0.,0.],[.06,0.]],1.)
        self.session.begin(plan,Pose(1,0,0,0),1,self.grid)
        self.assertEqual(self.session.advance(Pose(1.1,0,0,0),1.1,self.grid),'OBSERVATION_REGION_EXHAUSTED')
        self.assertNotEqual(self.session.result,'OBSERVATION_COMPLETE')


if __name__ == '__main__':
    unittest.main()
