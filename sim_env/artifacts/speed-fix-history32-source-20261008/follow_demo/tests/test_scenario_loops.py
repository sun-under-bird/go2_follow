"""检验连续循环刺激和完成度的反例，防止转折丢步长、瞬移及假通过。"""
import math
import unittest
import threading
import numpy as np
from follow_demo.simulation import Simulation
from follow_demo.app import Command
from follow_demo.scenarios import SCENARIOS
from follow_demo.scenario_route import RouteWalker, route_points
from follow_demo.scenario_acceptance import ordered_gate_progress, loop_completion
from follow_demo.tests.check_navigation_demo import validate_stimulus


class ScenarioLoopTests(unittest.TestCase):
    """只检查路线逻辑与验收，不启动ROS或仿真后台服务。"""

    def test_square_has_four_right_angles_and_eight_meter_sides(self):
        """闭合的四连弯必须包含四个直角及转弯后的长直行段。"""
        points = route_points(SCENARIOS['square_loop'])
        directions = [(b[0]-a[0], b[1]-a[1]) for a,b in zip(points,points[1:])]
        right_angles = sum(abs(a[0]*b[0]+a[1]*b[1]) < 1e-9 for a,b in zip(directions,directions[1:]+directions[:1]))
        self.assertEqual(right_angles, 4)
        self.assertEqual(RouteWalker(SCENARIOS['square_loop']).length, 32.0)
        self.assertEqual(points[0], points[-1])

    def test_large_step_keeps_remainder_and_multiple_laps(self):
        """一次跨过多个拐角和圈接缝，路程仍是完整的速度积分。"""
        walker = RouteWalker(SCENARIOS['square_loop'])
        self.assertEqual(walker.advance(.5, 140), [8., 0.])
        self.assertEqual(walker.status()['laps'], 2)
        self.assertAlmostEqual(walker.status()['progress_m'], 70)

    def test_step_partition_and_speed_change_preserve_timing(self):
        """同样总路程不受物理步长划分影响，调速不跳到累计时间的新位置。"""
        a, b = [RouteWalker(SCENARIOS['slalom_loop']) for _ in range(2)]
        for _ in range(1000):
            a.advance(.5, .07)
        b.advance(.5, 70)
        for x,y in zip(a.position,b.position):
            self.assertAlmostEqual(x,y, places=8)
        before = a.position.copy()
        a.advance(.7, 0)
        self.assertEqual(a.position, before)
        a.advance(.2, .5)
        self.assertLessEqual(math.dist(before,a.position), .100001)

    def test_finite_route_still_stops_at_exact_endpoint(self):
        """基础场景仍保留有限终点，不因新增循环而改变旧测试语义。"""
        walker = RouteWalker(SCENARIOS['corner'])
        self.assertEqual(walker.advance(.5, 1000), [7.5,1.6])
        self.assertTrue(walker.status()['finished'])

    def test_approach_does_not_teleport_or_count_as_lap(self):
        """人工偏离路线后连续返回起点，不能瞬移或凭返回距离虚增圈数。"""
        walker = RouteWalker(SCENARIOS['square_loop'], [2.,-1.])
        self.assertEqual(walker.advance(.5,1),[2.,-.5])
        self.assertEqual(walker.status()['progress_m'],0)
        self.assertEqual(walker.advance(.5,2),[2.5,0.])

    def test_target_route_does_not_cross_closed_obstacle_boxes(self):
        """逐段检查合成目标路线与箱体的交集，包含第二圈接缝及外侧返程。"""
        for name in ('square_loop','slalom_loop','wall_loop'):
            points = route_points(SCENARIOS[name])
            for a,b in zip(points,points[1:]):
                for x,y,sx,sy,_ in SCENARIOS[name]['boxes']:
                    low,high=0.,1.
                    for axis,center,half in ((0,x,sx+.1),(1,y,sy+.1)):
                        delta=b[axis]-a[axis]
                        if abs(delta)<1e-12:
                            if not center-half <= a[axis] <= center+half:
                                high=-1
                                break
                        else:
                            first,second=sorted(((center-half-a[axis])/delta,(center+half-a[axis])/delta))
                            low,high=max(low,first),min(high,second)
                    self.assertGreater(low,high,msg=f'{name}: 路段 {a} -> {b} 穿过障碍')

    def rows(self):
        """生成独立按时间推进的两圈刺激，机器人位置默认固定，专供反例。"""
        walker=RouteWalker(SCENARIOS['square_loop'])
        result=[]
        for t in range(139):
            position=walker.advance(.5,0 if t==0 else 1)
            result.append(dict(t=float(t),scenario='square_loop',epoch=1,target=position,target_speed=.5,
                               target_mode='route',target_route=walker.status(),x=0.,y=0.,distance=2.))
        return result

    def test_loop_stimulus_two_laps_and_hold_are_distinguished(self):
        """时序正确的两圈刺激通过；提前停人即使接近接缝也必须失败。"""
        rows=self.rows()
        baseline=dict(t=0.,epoch=1,target=[2.,0.])
        self.assertTrue(validate_stimulus(rows,baseline,'square_loop',.5,138)['valid'])
        rows[-1]['target_mode']='hold'
        self.assertFalse(validate_stimulus(rows,baseline,'square_loop',.5,138)['valid'])

    def test_fake_loop_counter_fails_stimulus(self):
        """位置正确但进度或圈数伪造时也拒收，避免显示计数替代时序验收。"""
        rows=self.rows()
        rows[-1]['target_route']['laps']=20
        self.assertFalse(validate_stimulus(rows,dict(t=0.,epoch=1,target=[2.,0.]),'square_loop',.5,138)['valid'])

    def test_stationary_robot_is_not_credited_with_human_laps(self):
        """人持续走两圈、狗原地等待，不能靠目标计数或净位移条件冒充完成。"""
        result=loop_completion(self.rows(),route_points(SCENARIOS['square_loop']),2,dict(checks=dict(physical_progress=True)))
        self.assertFalse(result['met'])
        self.assertEqual(result['robot']['completed_laps'],0)

    def test_ordered_gates_credit_two_circuits_but_reject_reverse(self):
        """依次实际经过所有路段才累计圈数，反向或只在一个门附近抖动不计圈。"""
        points=route_points(SCENARIOS['square_loop'])
        gates=[[(a[0]+b[0])/2,(a[1]+b[1])/2] for a,b in zip(points,points[1:])]
        gates[-1]=points[-1]
        rows=[dict(t=i,x=p[0],y=p[1]) for i,p in enumerate(gates*2)]
        self.assertEqual(ordered_gate_progress(rows,points)['completed_laps'],2)
        self.assertLess(ordered_gate_progress(list(reversed(rows)),points)['completed_laps'],2)
        self.assertEqual(ordered_gate_progress([rows[0]]*100,points)['completed_laps'],0)

    def test_safe_corner_cut_returns_to_start_region_without_short_segment_midpoint(self):
        """完整走过四边后在通道内切弯回到起点区域，不能因人为拆段而漏算一圈。"""
        points=route_points(SCENARIOS['square_loop'])
        positions=[[5.,0.],[8.,4.],[4.,8.],[0.,4.],[1.7,1.3]]
        rows=[dict(t=i,x=p[0],y=p[1]) for i,p in enumerate(positions)]
        self.assertEqual(ordered_gate_progress(rows,points)['completed_laps'],1)
        # 只有起点附近的轨迹，仍不能跳过前面的四边。
        self.assertEqual(ordered_gate_progress(rows[-1:]*20,points)['completed_laps'],0)

    def test_approaching_gate_without_crossing_plane_is_not_progress(self):
        """只接近路段中点、还没有沿行走方向越过截面，不能提前计入进度。"""
        points=route_points(SCENARIOS['square_loop'])
        rows=[dict(t=i,x=3.5,y=.1) for i in range(20)]
        self.assertEqual(ordered_gate_progress(rows,points)['passed_gates'],0)

    def test_invalid_geometry_and_time_fail_before_motion(self):
        """非法坐标、零长度路线与非法步长不能进入物理线程。"""
        for spec in (dict(route=[[2.,0.]]),dict(route=[[float('nan'),0.]])):
            with self.assertRaises(ValueError):
                RouteWalker(spec)
        walker=RouteWalker(SCENARIOS['square_loop'])
        with self.assertRaises(ValueError):
            walker.advance(.5,-1)

    def test_route_button_and_hold_resume_keep_current_progress(self):
        """重复点击路线、调速和停人后续走，不能悄悄返回第一段或清空圈数。"""
        receiver=Simulation.__new__(Simulation)
        receiver.lock=threading.Lock()
        receiver.target=np.array([2.,0.])
        receiver.target_mode='hold'
        receiver.target_speed=.5
        receiver.route_walker=None
        receiver.scenario='square_loop'
        receiver.action('route')
        walker=receiver.route_walker
        receiver.target[:]=walker.advance(.5,65)
        receiver.action('route',speed=.7)
        self.assertIs(receiver.route_walker,walker)
        self.assertEqual(walker.status()['laps'],1)
        receiver.action('hold')
        receiver.action('route')
        self.assertIs(receiver.route_walker,walker)
        self.assertEqual(walker.status()['progress_m'],32.5)

    def test_other_movement_discards_old_route_and_api_accepts_new_scenes(self):
        """自由走动后接入路线使用新位置；API目录覆盖每个新增循环场景。"""
        receiver=Simulation.__new__(Simulation)
        receiver.lock=threading.Lock()
        receiver.target=np.array([2.,0.])
        receiver.target_mode='hold'
        receiver.target_speed=.5
        receiver.route_walker=RouteWalker(SCENARIOS['square_loop'])
        receiver.scenario='square_loop'
        receiver.action('straight')
        self.assertIsNone(receiver.route_walker)
        for name in ('square_loop','slalom_loop','wall_loop'):
            self.assertEqual(Command(action='scenario',scenario=name).scenario,name)


if __name__ == '__main__':
    unittest.main()
