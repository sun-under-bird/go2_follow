"""验证矩形直行、旋转扫角、内部障碍、姿态搜索和统一速度边界。"""
import math
import unittest
from unittest.mock import patch
import numpy as np
from follow_demo.controller import Pose
from follow_demo.local_map import RollingMap
from follow_demo.local_planner import LocalPlanner
from follow_demo.navigation import NavigationController
from follow_demo.navigation_config import MAX_NAVIGATION_SPEED, MAX_NAVIGATION_TURN
from follow_demo.mppi_runtime import parameters
from follow_demo.footprint import polygon_cells
from follow_demo.observation import ObservationSession
from follow_demo.observation_geometry import observation_view,check_observation_position
from follow_demo.local_planner import Plan


class RectangleFootprintTests(unittest.TestCase):
    """使用实际尺寸的自由证据测试，不能以中心线无碰撞代替全机身。"""
    def grid(self,resolution=.02,size=201):
        """构造以零点为中心的静态已知地图。"""
        grid = RollingMap(size=size,resolution=resolution,static_history=True)
        grid.origin = np.full(2,-size*resolution/2)
        grid.seen[:] = 1.
        grid.last_depth = 1.
        return grid

    def test_requested_dimensions_and_command_limits(self):
        """Nav2 与最终控制共享用户指定尺寸和上限。"""
        grid = self.grid()
        self.assertEqual((grid.footprint.length,grid.footprint.width),(.7,.32))
        config = parameters()
        controller = config['/go2_follow_mppi/controller_server']['ros__parameters']
        self.assertEqual(controller['FollowPath']['vx_max'],.8)
        self.assertEqual(controller['FollowPath']['wz_max'],1.)
        self.assertTrue(controller['FollowPath']['CostCritic']['consider_footprint'])
        self.assertEqual((MAX_NAVIGATION_SPEED,MAX_NAVIGATION_TURN),(.8,1.))

    def test_observation_keeps_rectangle_admission_in_narrow_channel(self):
        """0.60m直通道已容纳含漂移余量的矩形，观察阶段不能再用中心距离把它拒绝。"""
        grid = RollingMap(size=81,resolution=.1,static_history=True)
        grid.origin[:] = [-4.,-4.]
        x,y = grid.centers()
        grid.seen[(abs(x)<1.1)&(abs(y)<.3)] = 1.
        free,_,_ = grid.layers(1.)
        expected = observation_view(grid,free,[0.,0.],0.)['unknown_cells']
        check = check_observation_position(grid,[0.,0.],0.,expected,1.,initial_yaw=0.)
        self.assertTrue(check['valid'])
        self.assertTrue(check['footprint_clear'])
        self.assertLess(check['clearance'],.16+.1*math.sqrt(.5)+.1)
        plan = Plan('OBSERVING','GOAL_UNOBSERVED',path=[[0.,0.]],look_yaw=0.,
                    observation_region=[[0.,0.]],observation_cells=list(expected))
        pose = Pose(1.,0.,0.,0.)
        session = ObservationSession()
        session.begin(plan,pose,1.,grid)
        session.stage = 'ORIENT'
        self.assertEqual(session.advance(pose,1.,grid),'OBSERVATION_EVALUATE')
        self.assertEqual(session.stage,'EVALUATE')

    def test_narrow_channel_accepts_straight_body_but_rejects_crosswise(self):
        """0.50 m 通道容纳宽0.32的矩形，但不能容纳横过来的长0.70。"""
        grid = self.grid()
        _,y = grid.centers()
        free = abs(y)<.25
        self.assertTrue(grid.pose_clear(free,0,0,0))
        self.assertFalse(grid.pose_clear(free,0,0,math.pi/2))
        self.assertTrue(grid.motion_clear(free,[0,0,0],[.5,0,0]))
        self.assertFalse(grid.motion_clear(free,[0,0,0],[0,0,math.pi/2]))

    def test_rotation_cannot_skip_obstacle_between_safe_endpoints(self):
        """原地0°和90°均安全，45°扫过的角点仍必须拒绝。"""
        grid = self.grid()
        free = np.ones_like(grid.occupied)
        free[grid.cell(.23,.23)] = False
        self.assertTrue(grid.pose_clear(free,0,0,0))
        self.assertTrue(grid.pose_clear(free,0,0,math.pi/2))
        self.assertFalse(grid.motion_clear(free,[0,0,0],[0,0,math.pi/2]))

    def test_internal_obstacle_is_checked_not_only_four_corners(self):
        """机身内部的小障碍即使不碰四角也不能漏掉。"""
        grid = self.grid()
        free = np.ones_like(grid.occupied)
        free[grid.cell(.1,0)] = False
        self.assertFalse(grid.pose_clear(free,0,0,0))

    def test_translation_sweep_detects_thin_obstacle(self):
        """首尾矩形都安全，长平移中间的单格障碍仍必须被扫掠覆盖。"""
        grid = self.grid()
        free = np.ones_like(grid.occupied)
        free[grid.cell(.55,0)] = False
        self.assertTrue(grid.pose_clear(free,0,0,0))
        self.assertTrue(grid.pose_clear(free,1.1,0,0))
        self.assertFalse(grid.motion_clear(free,[0,0,0],[1.1,0,0]))

    def test_unknown_and_window_outside_are_not_free(self):
        """矩形任意部分触及未知或窗口外都拒绝。"""
        grid = self.grid()
        free = np.ones_like(grid.occupied)
        free[grid.cell(.3,0)] = False
        self.assertFalse(grid.pose_clear(free,0,0,0))
        self.assertFalse(grid.pose_clear(np.ones_like(free),2.,0,0))

    def test_lattice_search_uses_heading_to_pass_channel(self):
        """无需转身的窄通道可取得进展，二维并集不会误许原地横转。"""
        grid = self.grid(.1,81)
        x,y = grid.centers()
        grid.seen[abs(y)>.25] = -np.inf
        free,allowed,clearance = grid.layers(1.)
        plan = LocalPlanner().search(grid,allowed,clearance,Pose(1.,0.,0.,0.),[5.,0.],[.5,0.],1.8)
        self.assertEqual(plan.kind,'FOLLOWING')
        self.assertGreater(plan.path[-1][0],.8)
        self.assertTrue(grid.route_clear(free,plan.path,0.))
        self.assertFalse(grid.route_clear(free,[[0,0]],0.,math.pi/2))

    def test_execution_guard_checks_rotation_even_without_translation(self):
        """前速零也要校验旋转角点及残余运动。"""
        grid = self.grid()
        free = np.ones_like(grid.occupied)
        free[grid.cell(.23,.23)] = False
        core = NavigationController()
        self.addCleanup(core.close)
        core.grid = grid
        self.assertFalse(core.braking_safe(Pose(1.,0.,0.,0.),(0.,1.),free)[0])

    def test_fast_pose_check_matches_complete_contact_cells(self):
        """随机位置、朝向和薄障碍下，快速判定必须等价于完整多边形接触格基准。"""
        grid = self.grid(.05,81)
        random = np.random.default_rng(20261005)
        for case in range(500):
            free = random.random(grid.occupied.shape)>.01
            x,y = random.uniform(-1.9,1.9,2)
            yaw = random.uniform(-math.pi,math.pi)
            cells = polygon_cells(grid.footprint.vertices(x,y,yaw),grid.origin,grid.resolution,grid.size)
            expected = cells is not None and bool(np.all(free[cells[:,0],cells[:,1]]))
            self.assertEqual(grid.pose_clear(free,x,y,yaw),expected,msg=f'随机反例{case}')

    def test_motion_fast_proof_matches_original_rectangle_sweep(self):
        """随机平移/旋转中，外接框充分条件必须与禁用快速证明的原矩形算法一致。"""
        grid = self.grid(.05,81)
        random = np.random.default_rng(20261006)
        for case in range(200):
            free = random.random(grid.occupied.shape) > (.001 if case%2 else .015)
            a = [*random.uniform(-1.8,1.8,2),random.uniform(-math.pi,math.pi)]
            b = [a[0]+random.uniform(-.4,.4),a[1]+random.uniform(-.4,.4),a[2]+random.uniform(-1.,1.)]
            actual = grid.motion_clear(free,a,b)
            with patch.object(grid,'window_free',return_value=False):
                expected = grid.motion_clear(free,a,b)
            self.assertEqual(actual,expected,msg=f'扫掠随机反例{case}')

    def test_writable_window_query_observes_in_place_change(self):
        """外部掩码原地新增单格障碍后，快速查询不能沿用旧自由结果。"""
        grid = self.grid()
        free = np.ones_like(grid.occupied)
        self.assertTrue(grid.window_free(free,50,50,60,60))
        free[55,55] = False
        self.assertFalse(grid.window_free(free,50,50,60,60))


if __name__ == '__main__':
    unittest.main()
