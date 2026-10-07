"""验证观察路径在中断和地图冲突后失效，旧异步结果不能让机器人重新启动。"""
import copy
import unittest
from concurrent.futures import Future
from unittest.mock import patch
import numpy as np
from follow_demo.controller import Pose
from follow_demo.local_planner import Plan
from follow_demo.navigation import NavigationController


class DeferredExecutor:
    """只保存提交请求，由用例决定何时完成，不创建线程或执行实际地图搜索。"""
    def __init__(self):
        """记录尚未返回的搜索，使测试可以确定地复现中断期间的竞争窗口。"""
        self.requests = []

    def submit(self, function, *args, **kwargs):
        """模拟已经开始运行、无法通过 cancel 撤销的规划任务。"""
        future = Future()
        # 真正危险的是已经运行的搜索：停止命令不能保证该搜索也立即退出。
        future.set_running_or_notify_cancel()
        self.requests.append((future, function, args, kwargs))
        return future

    def shutdown(self, wait=True, cancel_futures=False):
        """兼容控制器清理接口；这里没有真实线程或服务需要等待。"""
        if cancel_futures:
            for future, _, _, _ in self.requests:
                future.cancel()


class NavigationLifecycleTests(unittest.TestCase):
    """以输入事件、地图变化和延迟返回构造可重现的生命周期反例。"""
    def fresh_inputs(self, core, now):
        """恢复同一位置的新里程计、UWB 和深度，排除输入超时对结果的干扰。"""
        core.history.add(Pose(now, 0., 0., 0.))
        if core.target_stamp != now:
            self.assertTrue(core.observe(4., 0., now))
        core.grid.last_depth = now

    def observing_controller(self, now=10.):
        """构造仍在接近观察点的任务及有效旧 MPPI 候选，替换唯一搜索线程。"""
        executor = DeferredExecutor()
        with patch('follow_demo.navigation.ThreadPoolExecutor', return_value=executor):
            core = NavigationController()
        self.addCleanup(core.close)
        self.fresh_inputs(core, now)
        core.grid.confirmed = True
        core.grid.seen[:] = now
        core.plan = Plan('OBSERVING', 'GOAL_UNOBSERVED', [[0., 0.], [1.5, 0.]], 1.,
                         observation_region=[[1.55, .05]])
        core.observation.begin(core.plan, core.history.values[-1], now, core.grid)
        core.last_plan = now
        core.external_active, core.external_stamp = True, now
        core.external_command = (.35, .6)
        core.command, core.acceleration = [.35, .6], [.2, .2]
        return core, executor

    def start_pending_search(self, core, now):
        """通过正常规划入口提交一个运行中的候选，随后由测试控制其返回时刻。"""
        core.last_plan = now - 1.
        core.update_plan(core.history.values[-1], np.asarray(core.target),
                         core.target_velocity, core.desired_distance, now)
        self.assertIsNotNone(core.future)
        self.assertFalse(core.future.done())
        return core.future

    def assert_observation_revoked(self, core, revision):
        """路径、观察任务与动作候选必须同时撤销，单纯归零当前输出还不够。"""
        self.assertFalse(core.plan.path)
        self.assertNotEqual(core.plan.kind, 'OBSERVING')
        self.assertIsNone(core.observation.plan)
        self.assertEqual(core.observation.stage, 'IDLE')
        self.assertFalse(core.external_active)
        self.assertFalse(core.tracking_requested)
        self.assertGreater(core.plan_revision, revision)
        self.assertEqual(tuple(core.command), (0., 0.))

    def test_input_interruptions_revoke_observation_and_mppi_candidate(self):
        """暂停及各类关键输入中断都不能留下 IDLE 观察器和可执行旧观察路径。"""
        cases = (
            ('暂停', 'PAUSED', dict(enabled=False), None),
            ('急停', 'ESTOP', dict(emergency=True), None),
            ('UWB 无效', 'TARGET_LOST', dict(signal_valid=False), None),
            ('UWB 超时', 'TARGET_LOST', {}, 'target'),
            ('姿态未就绪', 'INITIALIZING', dict(ready=False), None),
            ('里程计丢失', 'ODOM_LOST', {}, 'odom'),
            ('深度过期', 'WAITING', {}, 'depth'),
        )
        for name, state, arguments, missing in cases:
            with self.subTest(interruption=name):
                core, _ = self.observing_controller()
                revision = core.plan_revision
                if missing == 'target':
                    core.target_stamp = 9.
                elif missing == 'odom':
                    core.history.values.clear()
                elif missing == 'depth':
                    core.grid.last_depth = 8.
                self.assertEqual(core.step(10., **arguments), (0., 0.))
                self.assertEqual(core.state, state)
                if missing == 'depth':
                    self.assertEqual(core.code, 'MAP_STALE')
                self.assert_observation_revoked(core, revision)

    def test_resume_rejects_pre_interruption_future_but_accepts_new_search(self):
        """即使旧搜索仍满足普通时效门槛，恢复后也不能将它当作新一轮观察任务。"""
        core, executor = self.observing_controller()
        old_plan = copy.deepcopy(core.plan)
        old_future = self.start_pending_search(core, 10.)
        self.assertEqual(core.step(10.02, enabled=False), (0., 0.))
        revision = core.plan_revision
        # 旧候选只晚到 0.08 s，普通的 0.8 s 时效检查无法识别这次中断。
        old_future.set_result((old_plan, 20., 10.))
        self.fresh_inputs(core, 10.08)
        self.assertEqual(core.step(10.08), (0., 0.))
        self.assertFalse(core.plan.path)
        self.assertIsNone(core.observation.plan)
        self.assertFalse(core.tracking_requested)
        self.assertEqual(core.plan_revision, revision)
        self.assertGreater(len(executor.requests), 1)
        self.assertIsNot(core.future, old_future)

        # 失效屏障只拒绝中断前的任务，不能让恢复后的正常路线永久无法接入。
        new_plan = Plan('FOLLOWING', 'FOLLOW_REGION_REACHABLE', [[0., 0.], [2., 0.]])
        core.future.set_result((new_plan, 20., 10.08))
        self.fresh_inputs(core, 10.12)
        core.step(10.12)
        self.assertEqual(core.plan.kind, 'FOLLOWING')
        self.assertEqual(core.plan.path, new_plan.path)
        self.assertGreater(core.plan_revision, revision)

    def test_path_invalid_cannot_replay_observation_path_or_late_candidate(self):
        """新障碍切断路线后，残留观察路径和冲突前搜索结果都不能再次驱动动作。"""
        core, _ = self.observing_controller()
        old_plan = copy.deepcopy(core.plan)
        old_future = self.start_pending_search(core, 10.)
        revision = core.plan_revision
        # 障碍位于路线中段，当前机身及零实测速度的停车包络仍然安全。
        core.grid.occupied[core.grid.cell(.95, 0.)] = True
        self.fresh_inputs(core, 10.02)
        self.assertEqual(core.step(10.02), (0., 0.))
        self.assertEqual(core.code, 'PATH_INVALID')
        self.assert_observation_revoked(core, revision)

        old_future.set_result((old_plan, 20., 10.))
        self.fresh_inputs(core, 10.06)
        # 连续收到旧路线的动作候选也不能代替一条重新验证后的新路径。
        core.external_active, core.external_stamp = True, 10.06
        core.external_command = (.35, .6)
        self.assertEqual(core.step(10.06), (0., 0.))
        self.assertFalse(core.plan.path)
        self.assertIsNone(core.observation.plan)
        self.assertFalse(core.tracking_requested)

    def test_blocked_future_cannot_replace_valid_plan_or_begin_observation(self):
        """候选接入前必须通过新地图校验，拒绝过程不能破坏仍有效的执行任务。"""
        for candidate_kind in ('FOLLOWING', 'OBSERVING'):
            with self.subTest(candidate=candidate_kind):
                core, _ = self.observing_controller()
                if candidate_kind == 'OBSERVING':
                    # 没有观察任务时，非法观察候选也不能先 begin 再由 step 撤销。
                    core.plan = Plan('FOLLOWING', 'FOLLOW_REGION_REACHABLE', [[0., 0.], [1.5, 0.]])
                    core.observation.clear()
                old_plan = copy.deepcopy(core.plan)
                old_observation = copy.deepcopy(core.observation.plan)
                old_sequence, old_revision = core.observation.sequence, core.plan_revision
                candidate = Plan(candidate_kind, 'GOAL_UNOBSERVED' if candidate_kind == 'OBSERVING'
                                 else 'FOLLOW_REGION_REACHABLE',
                                 [[0., 0.], [1.2, .8], [2.2, .8]], 1.,
                                 observation_region=[[2.25, .85]])
                future = self.start_pending_search(core, 10.)
                # 障碍切断异步候选的中段，但距旧直线路线 0.8 m，旧任务仍然安全。
                core.grid.occupied[core.grid.cell(1.2, .8)] = True
                self.fresh_inputs(core, 10.06)
                _, allowed, _ = core.grid.layers(10.06)
                self.assertTrue(core.path_ahead(core.history.values[-1], allowed))
                self.assertFalse(core.grid.segment_clear(allowed, candidate.path[0], candidate.path[1]))
                future.set_result((candidate, 20., 10.))
                core.update_plan(core.history.values[-1], np.asarray(core.target),
                                 core.target_velocity, core.desired_distance, 10.06)
                self.assertEqual(core.plan, old_plan)
                self.assertEqual(core.observation.plan, old_observation)
                self.assertEqual(core.observation.sequence, old_sequence)
                self.assertEqual(core.plan_revision, old_revision)

    def test_operator_transport_loss_revokes_observation_and_old_future(self):
        """ROS 通信看门狗直接调用 halt 时也需撤销任务，并隔离中断前运行中的搜索。"""
        core, _ = self.observing_controller()
        old_plan = copy.deepcopy(core.plan)
        old_future = self.start_pending_search(core, 10.)
        old_revision = core.plan_revision
        core.tracking_requested = True
        # 此入口不经过 step 更新 last_tick；必须使用通信中断发生时的真实控制时间。
        self.assertEqual(core.halt('INPUT_LOST', 'OPERATOR_STALE', '仿真状态通信中断', now=10.02),
                         (0., 0.))
        self.assertEqual(core.state, 'INPUT_LOST')
        self.assertEqual(core.code, 'OPERATOR_STALE')
        self.assert_observation_revoked(core, old_revision)

        old_future.set_result((old_plan, 20., 10.))
        self.fresh_inputs(core, 10.08)
        self.assertEqual(core.step(10.08), (0., 0.))
        self.assertFalse(core.plan.path)
        self.assertIsNone(core.observation.plan)
        self.assertFalse(core.tracking_requested)
        self.assertIsNot(core.future, old_future)


if __name__ == '__main__':
    unittest.main()
