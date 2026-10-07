"""验证未知区域、观测过期、绕障路线和停车约束的行为，不依赖 ROS 或物理引擎。"""
import math
import unittest
import numpy as np
from follow_demo.controller import Pose
from follow_demo.local_map import RollingMap
from follow_demo.local_planner import LocalPlanner, Plan, OBSERVATION_DRIFT_MARGIN, OBSERVATION_CANDIDATE_BUDGET
from follow_demo.navigation import NavigationController
from follow_demo.observation_geometry import (observation_view,check_observation_position,observation_cell_state,
                                             OBSERVATION_MIN_GAIN,OBSERVATION_MIN_OVERLAP)


class NavigationTests(unittest.TestCase):
    """使用明确构造的传感器或地图样本覆盖有限视野下的关键反例。"""
    def ready_controller(self, target=4.):
        """构造输入新鲜的通道，直接验证控制器边界；每个用例均回收搜索线程。"""
        core = NavigationController()
        self.addCleanup(core.close)
        core.history.add(Pose(1,0,0,0))
        core.observe(target,0,1)
        core.grid.seen[:] = 1
        core.grid.confirmed,core.grid.last_depth = True,1
        core.plan = Plan('FOLLOWING','FOLLOW_REGION_REACHABLE',[[0.,0.],[2.,0.]])
        core.last_plan = 1
        return core

    def test_arrival_uses_region_and_controller_tolerance(self):
        """人在舒适区边缘停下时进入保持，不因厘米级短路径反复启动 MPPI。"""
        core = self.ready_controller(2.04)
        self.assertEqual(core.step(1),(0.,0.))
        self.assertEqual(core.state,'HOLDING')
        self.assertFalse(core.tracking_requested)

    def test_arrival_cannot_be_claimed_through_wall(self):
        """直线距离足够近但中间有墙，仍不能把隔墙位置判作成功跟随。"""
        core = self.ready_controller(2.04)
        core.grid.occupied[core.grid.cell(1.1,0)] = True
        core.step(1)
        self.assertNotEqual(core.state,'HOLDING')

    def test_stationary_hold_does_not_require_robot_footprint_at_person(self):
        """原地保持时不驶向人的旁边未知格；当前包络和人与机器人间的自由视线仍必需。"""
        core = self.ready_controller(2.04)
        core.grid.seen[core.grid.cell(2.04,.3)] = -math.inf
        self.assertEqual(core.step(1),(0.,0.))
        self.assertEqual(core.state,'HOLDING')

    def test_no_progress_cannot_resume_due_to_later_map_growth(self):
        """探索失败后新栅格或旧候选不能自动启动，只有明确暂停才重新允许尝试。"""
        core = self.ready_controller()
        core.progress_failed = True
        core.external_active,core.external_stamp = True,1
        core.external_command = (.5,.3)
        self.assertEqual(core.step(1),(0.,0.))
        self.assertEqual(core.code,'NO_PROGRESS')
        core.step(1,enabled=False)
        self.assertFalse(core.progress_failed)

    def test_stale_mppi_cannot_replay_previous_command(self):
        """动作仍存活但候选速度过期时立即归零，同时允许重新获取控制输出。"""
        core = self.ready_controller()
        core.external_active,core.external_stamp = True,.6
        core.command,core.external_command = [.5,.1],(.5,.1)
        self.assertEqual(core.step(1),(0.,0.))
        self.assertEqual(core.code,'CONTROLLER_STALE')
        self.assertTrue(core.tracking_requested)

    def test_invalid_mppi_cannot_enter_smoother(self):
        """非有限候选不能经平滑器或裁剪函数变成任意运动命令。"""
        core = self.ready_controller()
        core.external_active,core.external_stamp = True,1
        core.external_command = (math.nan,.2)
        self.assertEqual(core.step(1),(0.,0.))
        self.assertEqual(core.code,'CONTROLLER_INVALID')

    def test_moving_person_does_not_trigger_stationary_hold(self):
        """跟随距离已经合适但人仍在走时，继续跟踪前方移动通道。"""
        core = self.ready_controller(2.04)
        core.target_velocity = [.5,0.]
        core.external_active,core.external_stamp = True,1
        core.external_command = (.5,0.)
        self.assertGreater(core.step(1)[0],0.)
        self.assertEqual(core.state,'FOLLOWING')

    def test_safe_narrow_corridor_is_not_disconnected_by_comfort_margin(self):
        """已验证的窄通道应可搜索，舒适偏好不能把安全路线误判为完全不可达。"""
        core = self.ready_controller()
        gx,gy = core.grid.centers()
        core.grid.seen[:] = -math.inf
        core.grid.seen[(abs(gy) < .61) & (gx > -2) & (gx < 5)] = 1
        plan,_,_ = core.compute_plan(core.grid,Pose(1,0,0,0),np.array([4.,0.]),[0.,0.],1.8,1.,[])
        self.assertEqual(plan.kind,'FOLLOWING')
        _,allowed,_ = core.grid.layers(1)
        self.assertTrue(all(core.grid.segment_clear(allowed,a,b) for a,b in zip(plan.path,plan.path[1:])))

    def test_unknown_is_not_drivable(self):
        """没有深度也没有人工确认时，未知地图不能产生可执行路线。"""
        grid = RollingMap()
        _, allowed, clearance = grid.layers(1)
        plan = LocalPlanner().search(grid, allowed, clearance, Pose(1, 0, 0, 0), [4, 0], [0, 0], 1.8)
        self.assertFalse(allowed.any())
        self.assertEqual(plan.reason, 'FOOTPRINT_UNOBSERVED')

    def test_expiration_never_erases_obstacles(self):
        """自由空间过期后回到未知，但障碍不能因超时被清除。"""
        grid = RollingMap()
        grid.seen[:] = 1
        grid.occupied[60, 70] = True
        free, allowed, _ = grid.layers(1 + grid.free_ttl + 1)
        self.assertFalse(free.any())
        self.assertFalse(allowed.any())
        self.assertTrue(grid.occupied[60, 70])

    def test_static_history_is_not_unknown_and_never_clears_obstacles(self):
        """静态模式保留旧净空，但从未观察区域和障碍仍禁止进入。"""
        grid = RollingMap(static_history=True)
        grid.confirm_start(0,0,1)
        grid.occupied[grid.cell(.5,0)] = True
        free,_,_ = grid.layers(100)
        self.assertTrue(free[grid.cell(0,0)])
        self.assertFalse(free[grid.cell(3,0)])
        self.assertFalse(free[grid.cell(.5,0)])
        self.assertEqual(grid.footprint_status(.2,0,100)['code'],'FOOTPRINT_OCCUPIED')

    def test_confirm_cannot_clear_observed_obstacle(self):
        """操作者确认不能覆盖相机已经观测到的近身障碍。"""
        grid = RollingMap()
        grid.recenter(0, 0)
        grid.occupied[grid.cell(0.5, 0)] = True
        self.assertFalse(grid.confirm_start(0, 0, 1))
        self.assertFalse(grid.confirmed)

    def test_rolling_window_preserves_world_locations(self):
        """窗口平移后旧墙仍处于相同世界位置，不能随机器人一起移动。"""
        grid = RollingMap()
        grid.occupied[grid.cell(2, 1)] = True
        grid.recenter(1.3, -0.7)
        self.assertTrue(grid.occupied[grid.cell(2, 1)])
        grid.recenter(30, 30)
        self.assertFalse(grid.occupied.any())

    def test_invalid_depth_does_not_create_free_space(self):
        """空洞、NaN 和超距不能作为清障观测。"""
        grid = RollingMap()
        grid.integrate(np.full((60, 106), np.nan), [60, 60, 53, 30], np.eye(3), np.zeros(3), 1)
        self.assertFalse(np.isfinite(grid.seen).any())
        self.assertIsNone(grid.last_depth)

    def test_front_depth_does_not_reveal_rear(self):
        """前方平面的理想深度可以确认前方，却不能生成相机后方自由空间。"""
        grid = RollingMap()
        rotation = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        grid.integrate(np.full((120, 212), 4.0), [108, 108, 106, 60], rotation, np.array([0.23, 0.05, 0.4]), 1)
        self.assertTrue(np.isfinite(grid.seen[grid.cell(2, 0)]))
        self.assertFalse(np.isfinite(grid.seen[grid.cell(-2, 0)]))
        self.assertTrue(grid.occupied[grid.cell(4.23, 0)])

    def test_ground_with_no_sky_return_remains_usable(self):
        """天空无返回不应阻断平地确认；但看见地面不能清掉已经存在的悬空障碍。"""
        grid = RollingMap()
        rotation = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        rows = np.indices((120, 212))[0]
        depth = np.where(rows > 60, 0.4 * 108 / np.maximum(rows - 60, 1), np.nan)
        wall = grid.cell(2.5, 0)
        grid.occupied[wall] = True
        grid.integrate(depth, [108, 108, 106, 60], rotation, np.array([0.23, 0.05, 0.4]), 1)
        self.assertTrue(np.isfinite(grid.seen[grid.cell(2.0, 0)]))
        self.assertTrue(grid.occupied[wall])

    def test_path_goes_around_long_wall(self):
        """已观察到障碍两端时，搜索应绕端点，不能因目标偏角大而改为直追目标。"""
        grid = RollingMap()
        grid.seen[:] = 1
        gx, gy = grid.centers()
        grid.occupied[(abs(gx - 2.5) < 0.15) & (abs(gy) < 1.4)] = True
        _, allowed, clearance = grid.layers(1)
        plan = LocalPlanner().search(grid, allowed, clearance, Pose(1, 0, 0, 0), [5.5, 0], [0, 0], 1.8)
        self.assertEqual(plan.kind, 'FOLLOWING')
        self.assertGreater(max(abs(p[1]) for p in plan.path), 1.8)
        self.assertTrue(all(grid.segment_clear(allowed, a, b) for a, b in zip(plan.path, plan.path[1:])))

    def test_hidden_goal_only_gets_known_observation_position(self):
        """目标在未观察区域时，允许先观察；路径终点及连线均必须在已知包络内。"""
        grid = RollingMap()
        grid.confirm_start(0, 0, 1)
        _, allowed, clearance = grid.layers(1)
        plan = LocalPlanner().search(grid, allowed, clearance, Pose(1, 0, 0, 0), [-4, 0], [0, 0], 1.8)
        self.assertEqual(plan.kind, 'OBSERVING')
        self.assertTrue(all(grid.permitted(allowed, *p) for p in plan.path))
        self.assertGreater(abs(plan.look_yaw), 1.5)

    def test_safe_corridor_without_turning_room_can_keep_moving(self):
        """窄通道可以安全前进时不强制停转；观察候选仍须额外转向漂移净空。"""
        grid = RollingMap(static_history=True)
        gx,gy = grid.centers()
        grid.seen[(abs(gy) < .31) & (gx > -2.) & (gx < 5.)] = 1.
        _,allowed,clearance = grid.layers(1.)
        before = allowed.copy()
        self.assertTrue(allowed[grid.cell(0.,0.)])
        plan = LocalPlanner().search(grid,allowed,clearance,Pose(1.,0.,0.,0.),[8.,0.],[0.,0.],1.8)
        self.assertEqual(plan.kind,'FOLLOWING')
        self.assertEqual(plan.reason,'KNOWN_CORRIDOR_PROGRESS')
        self.assertTrue(all(grid.segment_clear(allowed,a,b) for a,b in zip(plan.path,plan.path[1:])))
        cells = np.argwhere(allowed)
        points = grid.origin+(cells[:,::-1]+.5)*grid.resolution
        candidates = LocalPlanner.observation_candidates(grid,cells,points,
                    np.linalg.norm(points,axis=1),np.linalg.norm(points-[8.,0.],axis=1),
                    clearance,Pose(1.,0.,0.,0.))
        self.assertEqual(len(candidates),0)
        np.testing.assert_array_equal(allowed,before)

    def test_observation_candidates_include_noncoarse_nearby_viewpoint(self):
        """高净空近身视点不能被粗采样相位遗漏，边缘点不能挤占有界观察预算。"""
        grid = RollingMap(static_history=True)
        grid.confirm_start(0.,0.,1.)
        _,allowed,clearance = grid.layers(1.)
        cells = np.argwhere(allowed)
        points = grid.origin+(cells[:,::-1]+.5)*grid.resolution
        pose = Pose(1.,.14,-.05,0.)
        travel = np.linalg.norm(points-[pose.x,pose.y],axis=1)
        separation = np.linalg.norm(points-[4.,0.],axis=1)
        indices = LocalPlanner.observation_candidates(grid,cells,points,travel,separation,clearance,pose)
        self.assertLessEqual(len(indices),OBSERVATION_CANDIDATE_BUDGET)
        self.assertTrue(any(math.dist(point,[.15,-.05]) < 1e-6 for point in points[indices]))
        self.assertFalse(all(value % 3 == 0 for value in grid.cell(.15,-.05)))
        required = grid.radius+grid.resolution*math.sqrt(.5)+OBSERVATION_DRIFT_MARGIN
        self.assertTrue(np.all(clearance[cells[indices,0],cells[indices,1]] >= required))

    def test_refined_observation_does_not_repeat_tried_pose(self):
        """细化生成更多近身视点后，已执行位置和相近朝向仍不能重复占据观察任务。"""
        grid = RollingMap(static_history=True)
        grid.confirm_start(0.,0.,1.)
        _,allowed,clearance = grid.layers(1.)
        pose = Pose(1.,0.,0.,0.)
        planner = LocalPlanner()
        first = planner.search(grid,allowed,clearance,pose,[-4.,0.],[0.,0.],1.8)
        self.assertEqual(first.kind,'OBSERVING')
        tried = [[*first.path[-1],first.look_yaw]]
        second = planner.search(grid,allowed,clearance,pose,[-4.,0.],[0.,0.],1.8,tried)
        if second.path:
            angle = math.atan2(math.sin(second.look_yaw-first.look_yaw),math.cos(second.look_yaw-first.look_yaw))
            self.assertTrue(math.dist(first.path[-1],second.path[-1]) >= .2 or abs(angle) >= .35)

    def test_refined_observation_keeps_distant_corridor_representatives(self):
        """近身细化不能挤掉已知较远通道，否则绕长墙会退化成不断换近身短终点。"""
        grid = RollingMap(static_history=True)
        gx,gy = grid.centers()
        grid.seen[(abs(gx) < 3.) & (abs(gy) < 3.)] = 1.
        _,allowed,clearance = grid.layers(1.)
        cells = np.argwhere(allowed)
        points = grid.origin+(cells[:,::-1]+.5)*grid.resolution
        pose = Pose(1.,0.,0.,0.)
        travel = np.linalg.norm(points-[pose.x,pose.y],axis=1)
        separation = np.linalg.norm(points-[8.,0.],axis=1)
        indices = LocalPlanner.observation_candidates(grid,cells,points,travel,separation,clearance,pose)
        self.assertLessEqual(len(indices),OBSERVATION_CANDIDATE_BUDGET)
        self.assertTrue(np.any(travel[indices] > 1.))
        self.assertTrue(np.any(travel[indices] < .2))

    def test_observation_region_has_safe_related_views(self):
        """区域每个视点都需完整净空和相关收益，不能把任意到达圆当成可观察区域。"""
        grid = RollingMap(static_history=True)
        grid.confirm_start(0.,0.,1.)
        free,allowed,clearance = grid.layers(1.)
        plan = LocalPlanner().search(grid,allowed,clearance,Pose(1.,0.,0.,0.),[-4.,0.],[0.,0.],1.8)
        self.assertEqual(plan.kind,'OBSERVING')
        self.assertTrue(plan.observation_region)
        self.assertTrue(plan.observation_cells)
        for point in plan.observation_region:
            status = check_observation_position(grid,point,plan.look_yaw,plan.observation_cells,1.)
            self.assertTrue(status['valid'])
            self.assertGreaterEqual(status['gain'],OBSERVATION_MIN_GAIN)
            self.assertGreaterEqual(status['overlap_ratio'],OBSERVATION_MIN_OVERLAP)
        state = observation_cell_state(grid,plan.observation_cells,1.)
        self.assertFalse(state['known'])
        self.assertTrue(all(grid.permitted(allowed,*point) for point in plan.observation_region))

    def test_actual_position_requires_original_frontier_overlap(self):
        """位置安全且附近仍有未知，也不能用看向另一侧的收益完成原前沿观察。"""
        grid = RollingMap(static_history=True)
        grid.confirm_start(0.,0.,1.)
        free,_,_ = grid.layers(1.)
        expected = observation_view(grid,free,[0.,0.],math.pi)['unknown_cells']
        status = check_observation_position(grid,[.03,.02],0.,expected,1.)
        self.assertGreaterEqual(status['gain'],OBSERVATION_MIN_GAIN)
        self.assertFalse(status['valid'])
        self.assertEqual(status['reason'],'OBSERVATION_FRONTIER_MISMATCH')

    def test_revealed_frontier_remains_visible_without_predicted_new_gain(self):
        """相关格变已知后几何重叠仍保留，实际新增信息要与预测剩余收益分开。"""
        grid = RollingMap(static_history=True)
        grid.confirm_start(0.,0.,1.)
        free,_,_ = grid.layers(1.)
        expected = observation_view(grid,free,[0.,0.],0.)['unknown_cells']
        grid.seen[:] = 2.
        status = check_observation_position(grid,[0.,0.],0.,expected,2.)
        self.assertFalse(status['valid'])
        self.assertTrue(status['geometry_valid'])
        self.assertEqual(status['reason'],'OBSERVATION_LOW_GAIN')
        self.assertAlmostEqual(status['overlap_ratio'],1.)
        self.assertEqual(observation_cell_state(grid,expected,2.)['known'],expected)

    def test_observation_info_separates_wall_from_traversable_growth(self):
        """新障碍属于相关已知信息，但只有完整自由邻域才可计入可通行增长。"""
        grid = RollingMap(static_history=True)
        grid.confirm_start(0.,0.,1.)
        point = [2.05,.05]
        cell = grid.cell(*point)
        key = tuple(np.floor(np.asarray(point)/grid.resolution).astype(int))
        self.assertFalse(observation_cell_state(grid,[key],1.)['known'])
        grid.occupied[cell],grid.seen[cell] = True,2.
        wall = observation_cell_state(grid,[key],2.)
        self.assertEqual(wall['known'],{key})
        self.assertEqual(wall['occupied'],{key})
        self.assertFalse(wall['free'])
        self.assertFalse(wall['allowed'])
        gx,gy = grid.centers()
        grid.occupied[cell] = False
        grid.seen[(gx-point[0])**2+(gy-point[1])**2 < 1.**2] = 3.
        clear = observation_cell_state(grid,[key],3.)
        self.assertEqual(clear['free'],{key})
        self.assertEqual(clear['allowed'],{key})

    def test_observation_overlap_survives_subgoal_translation(self):
        """开阔前沿平移 0.11 m 仍应覆盖原前沿，不能因稀疏射线采样相位而失效。"""
        grid = RollingMap(static_history=True)
        grid.confirm_start(0.,0.,1.)
        free,_,_ = grid.layers(1.)
        reference = observation_view(grid,free,[.16,0.],1.)
        actual = check_observation_position(grid,[.05,0.],1.,reference['unknown_cells'],1.)
        self.assertTrue(actual['valid'])
        self.assertGreaterEqual(actual['overlap_ratio'],OBSERVATION_MIN_OVERLAP)

    def test_near_wall_blocks_observation_before_gain_range(self):
        """0.7 m 收益起点以前的已知墙同样必须遮挡，不能跨过近墙预测后方未知。"""
        grid = RollingMap(static_history=True)
        grid.confirm_start(0.,0.,1.)
        gx,gy = grid.centers()
        grid.occupied[(gx > .3) & (gx < .5) & (abs(gy) < .5)] = True
        free,_,_ = grid.layers(1.)
        view = observation_view(grid,free,[0.,0.],0.)
        self.assertEqual(view['gain'],0.)
        self.assertFalse(view['unknown_cells'])
        self.assertTrue(view['visible_cells'])

    def test_turning_person_does_not_send_robot_behind_itself(self):
        """人刚向左走时，机器人应接近可达跟随区域，不为抢占正后方而反向绕圈。"""
        grid = RollingMap()
        grid.seen[:] = 1
        _, allowed, clearance = grid.layers(1)
        plan = LocalPlanner().search(grid, allowed, clearance, Pose(1, 0, 0, 0), [2.2, 0.5], [0, 0.25], 1.9)
        self.assertEqual(plan.kind, 'FOLLOWING')
        self.assertGreaterEqual(plan.path[-1][1], -0.1)
        self.assertLess(math.dist(plan.path[0], plan.path[-1]), 0.7)

    def test_depth_loss_overrides_old_motion(self):
        """深度中断时，有效 UWB 和旧路线也不能维持运动。"""
        core = NavigationController()
        core.history.add(Pose(2, 0, 0, 0))
        core.observe(4, 0, 2)
        core.grid.confirm_start(0, 0, 0)
        core.grid.last_depth = 0.1
        core.command = [0.3, 0.2]
        self.assertEqual(core.step(2), (0.0, 0.0))
        self.assertEqual(core.code, 'MAP_STALE')

    def test_braking_reserves_space_before_boundary(self):
        """起点虽然可通行，高速所需的停车轨迹越界时仍必须被拒绝。"""
        core = NavigationController()
        core.grid.confirm_start(0, 0, 1)
        core.grid.last_depth = 1
        allowed, _, _ = core.grid.layers(1)
        safe, _ = core.braking_safe(Pose(1, 0.5, 0, 0), (0.48, 0), allowed)
        self.assertFalse(safe)
        safe, _ = core.braking_safe(Pose(1, 0, 0, 0), (0, 0.45), allowed)
        self.assertTrue(safe)

    def test_unobserved_rotation_is_rejected(self):
        """原地转身同样要求包络净空，不能绕过地图有效性。"""
        core = NavigationController()
        core.history.add(Pose(1, 0, 0, 0))
        core.observe(-4, 0, 1)
        core.grid.confirmed, core.grid.last_depth = True, 1
        self.assertEqual(core.step(1), (0.0, 0.0))
        self.assertEqual(core.code, 'FOOTPRINT_UNKNOWN')

    def test_forward_limit_preserves_safe_observation_turn(self):
        """前方停车空间不足时应减小前进，同时保留在已确认包络内的安全转向。"""
        core = NavigationController()
        core.grid.confirm_start(0, 0, 1)
        core.grid.last_depth = 1
        allowed, _, _ = core.grid.layers(1)
        command, _ = core.constrain_command(Pose(1, 0.5, 0, 0), (0.48, 0.45), allowed)
        self.assertLess(command[0], 0.48)
        self.assertEqual(command[1], 0.45)

if __name__ == '__main__':
    unittest.main()
