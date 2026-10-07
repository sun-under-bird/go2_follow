"""验证 spawn 搜索的数据隔离、结果接入屏障与任务进程回收，不启动 ROS 或仿真。"""
import copy
import math
import multiprocessing
import os
import threading
import unittest
from concurrent.futures import Future, ProcessPoolExecutor
from concurrent.futures.process import BrokenProcessPool
from unittest.mock import patch

import numpy as np

from follow_demo.controller import Pose
from follow_demo.local_map import RollingMap
from follow_demo.local_planner import Plan
from follow_demo.navigation import NavigationController, search_plan, search_worker_pid


class DeferredSearchExecutor:
    """保存提交参数；需要真进程时延迟转交，固定父状态改变与序列化之间的竞争窗口。"""
    def __init__(self, process_pool=None):
        """可包裹真实进程池，也可只提供可控 Future 来验证结果接入规则。"""
        self.process_pool = process_pool
        self.requests = []
        self.shutdown_arguments = None

    def submit(self, function, *args, **kwargs):
        """模拟无法取消的在途搜索，让调用者通过原 update_plan 接口提交。"""
        future = Future()
        if function is search_worker_pid:
            # 可控池不创建进程；预热只返回身份，不计入地图搜索请求。
            future.set_result(os.getpid())
            return future
        future.set_running_or_notify_cancel()
        self.requests.append((future, function, args, kwargs))
        return future

    def release(self):
        """将已保存的快照交给真实 spawn 池，并把结果转回导航器持有的 Future。"""
        future, function, args, kwargs = self.requests[-1]
        actual = self.process_pool.submit(function, *args, **kwargs)

        def receive(completed):
            """保留真实工作进程的结果或异常，使正常导航入口负责接入和拒收。"""
            try:
                result = completed.result()
            except Exception as error:
                future.set_exception(error)
            else:
                future.set_result(result)

        actual.add_done_callback(receive)
        return future

    def shutdown(self, wait=True, cancel_futures=False):
        """记录关闭语义并等待真实任务进程退出；无进程模式没有后台资源。"""
        self.shutdown_arguments = (wait, cancel_futures)
        if self.process_pool is not None:
            self.process_pool.shutdown(wait=wait, cancel_futures=cancel_futures)


class ProcessSearchTests(unittest.TestCase):
    """一次真实 spawn 验证隔离与退出，其余用确定性返回复现提交状态污染反例。"""
    def ready_controller(self, real_process=False):
        """构造完整已知的小地图及已提交偏好，所有资源均注册关闭清理。"""
        executor = DeferredSearchExecutor()
        if real_process:
            core = NavigationController(use_process_search=True)
            executor.process_pool = core.pool
            self.assertIsInstance(core.pool, ProcessPoolExecutor)
            self.assertEqual(core.pool._mp_context.get_start_method(), 'spawn')
            core.pool = executor
        else:
            with patch('follow_demo.navigation.ProcessPoolExecutor', return_value=executor):
                core = NavigationController(use_process_search=True)
        self.addCleanup(core.close)
        core.grid = RollingMap(size=61, static_history=True)
        core.grid.seen[:] = 10.
        core.grid.confirmed, core.grid.last_depth = True, 10.
        core.history.add(Pose(10., 0., 0., 0.))
        self.assertTrue(core.observe(4., 0., 10.))
        committed = Plan('FOLLOWING', 'COMMITTED', [[0., 0.], [1.5, 0.]])
        core.plan = copy.deepcopy(committed)
        core.planner.previous, core.planner.side = committed, 1
        return core, executor

    def submit_search(self, core, now=10.):
        """经正常 update_plan 提交，测试只控制异步结果的返回时间。"""
        core.last_plan = -math.inf
        core.update_plan(core.history.values[-1], np.asarray(core.target),
                         core.target_velocity, core.desired_distance, now)
        self.assertIsNotNone(core.future)
        self.assertFalse(core.future.done())
        return core.future

    def accept_result(self, core, now):
        """消费本次 Future，并禁止同一调用再提交任务以单独观察接入结果。"""
        core.last_plan = now
        core.update_plan(Pose(now, 0., 0., 0.), np.array([4., 0.]),
                         [0., 0.], core.desired_distance, now)

    def assert_safe_path(self, plan, grid, stamp):
        """逐段验证非空跟随路线仍受原地图包络限制，不以进程成功代替安全证据。"""
        self.assertEqual(plan.kind, 'FOLLOWING')
        self.assertGreaterEqual(len(plan.path), 2)
        self.assertTrue(np.isfinite(np.asarray(plan.path)).all())
        _, allowed, _ = grid.layers(stamp)
        self.assertTrue(all(grid.segment_clear(allowed, a, b)
                            for a, b in zip(plan.path, plan.path[1:])))

    def test_execution_frontier_requests_observation_only_after_slowdown(self):
        """几何路线存在但末级检测无续行出口时，减速后主动请求观察，不等待超时。"""
        for speed,expected in ((.3,False),(.05,True)):
            core,executor=self.ready_controller()
            core.continuation_limited=True
            core.motion=(speed,0.,0.)
            self.submit_search(core)
            self.assertEqual(executor.requests[-1][2][-1],expected)

    def test_spawn_does_not_pickle_controller_and_uses_private_inputs(self):
        """导航器含不可序列化锁仍能搜索，提交后的父地图和偏好修改不能进入工作进程。"""
        core, executor = self.ready_controller(real_process=True)
        # 若错误地提交绑定 core 的方法，spawn 序列化这个锁就会失败。
        core.unpicklable_lock = threading.Lock()
        original_grid = copy.deepcopy(core.grid)
        original_previous = copy.deepcopy(core.planner.previous)
        pose, target, velocity = core.history.values[-1], np.array([4., 0.]), [0., 0.]
        core.observation.tried = [[1., 1., 0.]]
        core.update_plan(pose, target, velocity, 1.8, 10.)
        future, function, args, kwargs = executor.requests[-1]
        self.assertIs(function, search_plan)
        self.assertEqual(kwargs, {})
        snapshot, submitted_pose, submitted_target, submitted_velocity = args[:4]
        submitted_tried, submitted_previous = args[6], args[9]
        self.assertIsNot(snapshot, core.grid)
        self.assertFalse(np.shares_memory(snapshot.seen, core.grid.seen))
        self.assertFalse(np.shares_memory(snapshot.occupied, core.grid.occupied))
        self.assertIsNot(submitted_pose, pose)
        self.assertFalse(np.shares_memory(submitted_target, target))
        self.assertIsNot(submitted_velocity, velocity)
        self.assertIsNot(submitted_previous.path, core.planner.previous.path)

        # 真正提交到进程池之前改变父数据，排除后台序列化先完成造成的偶然通过。
        core.grid.seen[:] = -math.inf
        core.grid.occupied[:] = True
        core.planner.previous.path[-1][:] = [100., 100.]
        pose.x, target[:], velocity[:] = 100., [100., 100.], [100., 100.]
        core.observation.tried[0][:] = [100., 100., 100.]
        np.testing.assert_array_equal(snapshot.seen, original_grid.seen)
        np.testing.assert_array_equal(snapshot.occupied, original_grid.occupied)
        self.assertEqual(submitted_previous, original_previous)
        self.assertEqual(submitted_pose.x, 0.)
        np.testing.assert_array_equal(submitted_target, [4., 0.])
        self.assertEqual(submitted_velocity, [0., 0.])
        self.assertEqual(submitted_tried, [[1., 1., 0.]])
        self.assertEqual(args[10], 1)

        executor.release()
        plan, wall_ms, requested = future.result(timeout=30.)
        self.assertNotEqual(plan.search_pid, os.getpid())
        self.assertEqual(requested, 10.)
        self.assertTrue(math.isfinite(wall_ms) and wall_ms >= 0.)
        self.assertTrue(math.isfinite(plan.search_cpu_ms) and plan.search_cpu_ms >= 0.)
        self.assert_safe_path(plan, original_grid, 10.)
        # 子进程恢复的 planner 状态也不能反向修改父进程保存的提交对象。
        self.assertEqual(submitted_previous, original_previous)
        np.testing.assert_array_equal(snapshot.seen, original_grid.seen)
        children = {child.pid: child for child in multiprocessing.active_children()}
        self.assertIn(plan.search_pid, children)
        worker = children[plan.search_pid]

        core.grid = original_grid
        self.accept_result(core, 10.1)
        self.assertEqual(core.plan, plan)
        self.assertEqual(core.planner.previous, plan)
        self.assertEqual(core.planner.side, plan.search_side)
        self.assertIsNot(core.plan, plan)
        self.assertIsNot(core.planner.previous, plan)
        self.assertIsNot(core.planner.previous.path, core.plan.path)
        accepted_path = copy.deepcopy(core.plan.path)
        plan.path[-1][:] = [100., 100.]
        self.assertEqual(core.plan.path, accepted_path)
        self.assertEqual(core.planner.previous.path, accepted_path)

        # close 必须同步等待退出，不能只发关闭请求就把验收资源留给下一轮。
        core.close()
        core.close()
        self.assertEqual(executor.shutdown_arguments, (True, True))
        self.assertTrue(core.search_closed)
        self.assertIsNone(core.future)
        self.assertNotIn(worker.pid, {child.pid for child in multiprocessing.active_children()})
        self.assertFalse(worker.is_alive())
        self.assertEqual(worker.exitcode, 0)

    def test_synchronous_interface_does_not_commit_search_state(self):
        """保留同步三元组入口；算出候选不能提前改变父规划器的已接入偏好。"""
        core, _ = self.ready_controller()
        previous = core.planner.previous
        saved = copy.deepcopy(previous)
        result = core.compute_plan(core.grid, Pose(10., 0., 0., 0.),
                                   np.array([4., 0.]), [0., 0.], 1.8, 10., [])
        self.assertEqual(len(result), 3)
        self.assert_safe_path(result[0], core.grid, 10.)
        self.assertEqual(result[0].search_pid, os.getpid())
        self.assertEqual(result[2], 10.)
        self.assertIs(core.planner.previous, previous)
        self.assertEqual(core.planner.previous, saved)
        self.assertEqual(core.planner.side, 1)

    def test_stale_results_do_not_commit_previous_or_side(self):
        """普通年龄过期与中断时间屏障都应丢弃候选，不能留下工作任务的终点和换边状态。"""
        for now, minimum in ((11., -math.inf), (10.1, 10.05)):
            with self.subTest(now=now, minimum=minimum):
                core, _ = self.ready_controller()
                previous, active_plan = core.planner.previous, core.plan
                saved = copy.deepcopy(previous)
                future = self.submit_search(core)
                candidate = Plan('FOLLOWING', 'RETURNED', [[0., 0.], [1.5, .8]], search_side=-1)
                future.set_result((candidate, 5., 10.))
                core.minimum_plan_stamp = minimum
                self.accept_result(core, now)
                self.assertEqual(core.rejected_plan_reason, 'SEARCH_RESULT_STALE')
                self.assertEqual(core.rejected_plan_count, 1)
                self.assertIs(core.plan, active_plan)
                self.assertIs(core.planner.previous, previous)
                self.assertEqual(core.planner.previous, saved)
                self.assertEqual(core.planner.side, 1)

    def test_new_obstacle_rejection_does_not_commit_previous_or_side(self):
        """新障碍切断返回路径时保留仍安全的旧路线及偏好，跟随和观察候选共用这道屏障。"""
        for kind in ('FOLLOWING', 'OBSERVING'):
            with self.subTest(kind=kind):
                core, _ = self.ready_controller()
                previous, active_plan = core.planner.previous, core.plan
                saved = copy.deepcopy(previous)
                future = self.submit_search(core)
                candidate = Plan(kind, 'RETURNED', [[0., 0.], [1.2, .8], [2.2, .8]],
                                 look_yaw=1., observation_region=[[2.25, .85]], search_side=-1)
                core.grid.occupied[core.grid.cell(1.2, .8)] = True
                _, allowed, _ = core.grid.layers(10.1)
                self.assertTrue(core.path_ahead(Pose(10.1, 0., 0., 0.), allowed))
                self.assertFalse(core.path_ahead(Pose(10.1, 0., 0., 0.), allowed, candidate))
                future.set_result((candidate, 5., 10.))
                self.accept_result(core, 10.1)
                self.assertEqual(core.rejected_plan_reason, 'RETURNED_PATH_INVALID')
                self.assertEqual(core.rejected_plan_count, 1)
                self.assertIs(core.plan, active_plan)
                self.assertIsNone(core.observation.plan)
                self.assertIs(core.planner.previous, previous)
                self.assertEqual(core.planner.previous, saved)
                self.assertEqual(core.planner.side, 1)

    def test_excluded_observation_does_not_commit_previous_or_side(self):
        """已执行过的观察候选即使路径安全，也不能成为下轮规划的终点或换边偏好。"""
        core, _ = self.ready_controller()
        previous, active_plan = core.planner.previous, core.plan
        saved = copy.deepcopy(previous)
        core.observation.tried = [[1.5, .8, 1.]]
        future = self.submit_search(core)
        candidate = Plan('OBSERVING', 'RETURNED', [[0., 0.], [1.5, .8]],
                         look_yaw=1., observation_region=[[1.5, .8]], search_side=-1)
        future.set_result((candidate, 5., 10.))
        self.accept_result(core, 10.1)
        self.assertIs(core.plan, active_plan)
        self.assertIsNone(core.observation.plan)
        self.assertIs(core.planner.previous, previous)
        self.assertEqual(core.planner.previous, saved)
        self.assertEqual(core.planner.side, 1)

    def test_failed_future_revokes_motion_and_prevents_resubmission(self):
        """任务异常和进程池损坏均安全断路，旧 MPPI 命令及重复提交不能恢复运动。"""
        for error in (TypeError('快照序列化失败'), BrokenProcessPool('工作进程退出')):
            with self.subTest(error=type(error).__name__):
                core, executor = self.ready_controller()
                previous = core.planner.previous
                saved = copy.deepcopy(previous)
                future = self.submit_search(core)
                core.external_active, core.external_stamp = True, 10.
                core.external_command, core.command = (.4, .2), [.4, .2]
                core.tracking_requested = True
                future.set_exception(error)
                for now in (10.1, 10.6):
                    core.history.add(Pose(now, 0., 0., 0.))
                    self.assertTrue(core.observe(4., 0., now))
                    core.grid.last_depth = now
                    self.assertEqual(core.step(now), (0., 0.))
                    self.assertEqual(core.code, 'SEARCH_WORKER_FAILED')
                    self.assertTrue(core.search_failed)
                    self.assertIsNone(core.future)
                    self.assertFalse(core.plan.path)
                    self.assertIsNone(core.observation.plan)
                    self.assertFalse(core.external_active)
                    self.assertFalse(core.tracking_requested)
                self.assertEqual(len(executor.requests), 1)
                self.assertEqual(core.rejected_plan_count, 1)
                self.assertEqual(core.rejected_plan_reason, 'SEARCH_WORKER_FAILED')
                self.assertIn(type(error).__name__, core.search_error)
                self.assertIs(core.planner.previous, previous)
                self.assertEqual(core.planner.previous, saved)
                self.assertEqual(core.planner.side, 1)

    def test_broken_pool_at_submit_also_revokes_motion(self):
        """进程池在 submit 当场报错时采用同一断路规则，不能只处理已返回 Future 的异常。"""
        core, executor = self.ready_controller()
        previous = core.planner.previous
        saved = copy.deepcopy(previous)
        core.external_active, core.external_stamp = True, 10.
        core.external_command, core.command = (.4, .2), [.4, .2]
        core.tracking_requested = True
        with patch.object(executor, 'submit', side_effect=BrokenProcessPool('提交前工作进程退出')) as submit:
            core.update_plan(core.history.values[-1], np.asarray(core.target),
                             core.target_velocity, core.desired_distance, 10.)
            self.assertEqual(submit.call_count, 1)
            self.assertTrue(core.search_failed)
            self.assertEqual(core.code, 'SEARCH_WORKER_FAILED')
            self.assertEqual(tuple(core.command), (0., 0.))
            self.assertFalse(core.external_active)
            self.assertFalse(core.tracking_requested)
            self.assertFalse(core.plan.path)
            self.assertIsNone(core.future)
            self.accept_result(core, 10.6)
            self.assertEqual(submit.call_count, 1)
        self.assertEqual(core.rejected_plan_count, 1)
        self.assertIn('BrokenProcessPool', core.search_error)
        self.assertIs(core.planner.previous, previous)
        self.assertEqual(core.planner.previous, saved)
        self.assertEqual(core.planner.side, 1)


if __name__ == '__main__':
    unittest.main()
