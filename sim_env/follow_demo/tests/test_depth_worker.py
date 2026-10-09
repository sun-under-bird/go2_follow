"""直接检查真实融合接入方法；只用完成 Future 与轻量上下文，不创建 ROS 节点或线程。"""
import copy
from concurrent.futures import Future
from types import MethodType, SimpleNamespace
import unittest

import numpy as np

from follow_demo.local_map import RollingMap
from follow_demo.controller import Pose, PoseHistory
from follow_demo.ros_nodes import FollowerNode, timestamp
from follow_demo.ros_contract import RosInterface


def depth_message(stamp):
    """生成最新帧缓存所需的最小消息，保持实际时间戳与米制深度编码。"""
    return SimpleNamespace(header=SimpleNamespace(stamp=timestamp(stamp), frame_id='camera_left_optical'),
                           width=4, height=4, encoding='32FC1', step=16, is_bigendian=False,
                           data=np.full((4, 4), 2.0, dtype='<f4').tobytes())


class DeferredPool:
    """记录提交参数但不执行任务，用于证明重算仍只保留一个在途 Future。"""
    def __init__(self):
        """保留检查用提交记录，不创建后台线程。"""
        self.calls = []

    def submit(self, function, *arguments):
        """返回未完成 Future，模拟当前唯一任务仍在运行。"""
        future = Future()
        self.calls.append((function, arguments, future))
        return future


class DepthWorkerTests(unittest.TestCase):
    """检查窗口、人工确认、代次与图像健康边界不能被旧地图副本覆盖。"""
    def context(self, stamp=5.0, integrated=True, error=None):
        """构造真实方法可用的最小状态，并让 Future 保存独立地图副本。"""
        grid = RollingMap(size=21, static_history=True)
        grid.last_depth = 4.0
        grid.frames = 2
        snapshot = copy.deepcopy(grid)
        snapshot.last_depth = stamp
        snapshot.frames += 1
        snapshot.version += 1
        snapshot.seen[0, 0] = stamp
        future = Future()
        if error is None:
            future.set_result((snapshot, integrated, 12.3))
        else:
            future.set_exception(error)
        errors = []
        node = SimpleNamespace(interface=RosInterface(), core=SimpleNamespace(grid=grid, depth_timeout=.9,
                                                    history=SimpleNamespace(values=[])),
                               operator={'epoch': 1}, epoch_start=3.0,
                               pending_depth=depth_message(stamp), depth_future=future,
                               depth_job={'epoch': 1, 'grid': grid, 'version': grid.version,
                                          'evidence_version': grid.evidence_version, 'stamp': stamp},
                               depth_worker_ms=0.0, depth_applied=0, depth_rejected=0,
                               depth_rejected_reason='', depth_worker_error='', depth_worker_closed=False,
                               get_logger=lambda: SimpleNamespace(error=errors.append))
        node.accept_depth_result = MethodType(FollowerNode.accept_depth_result, node)
        return node, grid, snapshot, errors

    def test_valid_result_is_adopted_once(self):
        """合法完成结果一次换图并清已处理缓存；重复取结果不能再次增加计数。"""
        node, _, snapshot, _ = self.context()
        FollowerNode.accept_depth_result(node, 5.2)
        self.assertIs(node.core.grid, snapshot)
        self.assertEqual(node.core.grid.last_depth, 5.0)
        self.assertEqual(node.depth_applied, 1)
        self.assertEqual(node.depth_worker_ms, 12.3)
        self.assertIsNone(node.pending_depth)
        self.assertIsNone(node.depth_future)
        self.assertIsNone(node.depth_job)
        FollowerNode.accept_depth_result(node, 5.3)
        self.assertEqual(node.depth_applied, 1)

    def test_newer_latest_frame_survives_valid_adoption(self):
        """在途任务完成时，新到缓存帧必须继续保留，不能被旧任务清掉。"""
        node, _, _, _ = self.context()
        latest = depth_message(5.1)
        node.pending_depth = latest
        FollowerNode.accept_depth_result(node, 5.2)
        self.assertIs(node.pending_depth, latest)
        self.assertEqual(node.depth_applied, 1)

    def test_unfinished_future_does_not_touch_live_map(self):
        """尚未完成任务只占据一个在途席位，控制线程不等待或提前接入。"""
        node, grid, _, _ = self.context()
        future = Future()
        node.depth_future = future
        job, latest = node.depth_job, node.pending_depth
        FollowerNode.accept_depth_result(node, 5.2)
        self.assertIs(node.core.grid, grid)
        self.assertIs(node.depth_future, future)
        self.assertIs(node.depth_job, job)
        self.assertIs(node.pending_depth, latest)

    def test_window_change_is_preserved(self):
        """单纯移动窗口允许按世界坐标接入；origin保留当前布局，新边缘仍为未知。"""
        node, grid, snapshot, _ = self.context()
        world = grid.point((10, 15))
        snapshot.seen[10, 15] = 5.
        snapshot.occupied[10, 15] = True
        grid.recenter(1.0, .4)
        origin = grid.origin.copy()
        FollowerNode.accept_depth_result(node, 5.2)
        self.assertIs(node.core.grid, snapshot)
        np.testing.assert_allclose(node.core.grid.origin, origin, atol=1e-12)
        cell = node.core.grid.cell(*world)
        self.assertIsNotNone(cell)
        self.assertTrue(node.core.grid.occupied[cell])
        self.assertEqual(node.core.grid.seen[cell], 5.)
        self.assertTrue(np.isneginf(node.core.grid.seen[:, -1]).all())
        self.assertEqual(node.depth_rebased, 1)
        self.assertEqual(node.depth_rejected, 0)

    def test_start_confirmation_is_preserved(self):
        """人工确认修改地图版本后，旧融合结果不能撤回已确认的起始自由证据。"""
        node, grid, _, _ = self.context()
        self.assertTrue(grid.confirm_start(0, 0, 5.1))
        confirmed_seen = grid.seen.copy()
        FollowerNode.accept_depth_result(node, 5.2)
        self.assertIs(node.core.grid, grid)
        self.assertTrue(grid.confirmed)
        np.testing.assert_array_equal(grid.seen, confirmed_seen)
        self.assertEqual(grid.last_depth, 4.0)
        self.assertEqual(node.depth_rejected_reason, 'DEPTH_MAP_CHANGED')

    def test_epoch_change_rejects_old_result(self):
        """场景代次变化使旧帧失效，即使版本号碰巧相同也不能接入。"""
        node, grid, _, _ = self.context()
        node.operator['epoch'] = 2
        FollowerNode.accept_depth_result(node, 5.2)
        self.assertIs(node.core.grid, grid)
        self.assertEqual(node.depth_rejected_reason, 'DEPTH_EPOCH_CHANGED')
        self.assertEqual(node.depth_applied, 0)

    def test_grid_identity_change_rejects_old_result(self):
        """同一代次中重建地图也须拒收旧对象的结果，不能只比较版本整数。"""
        node, _, _, _ = self.context()
        replacement = copy.deepcopy(node.core.grid)
        node.core.grid = replacement
        FollowerNode.accept_depth_result(node, 5.2)
        self.assertIs(node.core.grid, replacement)
        self.assertEqual(node.depth_rejected_reason, 'DEPTH_EPOCH_CHANGED')

    def test_capture_time_limits_reject_invalid_result(self):
        """超龄、早于代次起点与未来采集时间均不得刷新深度健康。"""
        for stamp, epoch_start, now in ((5.0, 3.0, 6.0), (5.0, 5.1, 5.2), (5.0, 3.0, 4.9)):
            with self.subTest(stamp=stamp, epoch_start=epoch_start, now=now):
                node, grid, _, _ = self.context(stamp)
                node.epoch_start = epoch_start
                FollowerNode.accept_depth_result(node, now)
                self.assertIs(node.core.grid, grid)
                self.assertEqual(grid.last_depth, 4.0)
                self.assertEqual(node.depth_rejected_reason, 'DEPTH_FRAME_STALE')

    def test_old_frame_cannot_replace_newer_depth(self):
        """同时间和更早帧均不能覆盖地图已经接纳的新深度。"""
        for last_depth in (5.0, 5.1):
            with self.subTest(last_depth=last_depth):
                node, grid, _, _ = self.context()
                grid.last_depth = last_depth
                FollowerNode.accept_depth_result(node, 5.2)
                self.assertIs(node.core.grid, grid)
                self.assertEqual(grid.last_depth, last_depth)
                self.assertEqual(node.depth_rejected_reason, 'DEPTH_FRAME_SUPERSEDED')

    def test_invalid_measurement_preserves_old_map(self):
        """后台判定没有有效测量时不换图，不能仅因任务完成刷新传感器健康。"""
        node, grid, _, _ = self.context(integrated=False)
        FollowerNode.accept_depth_result(node, 5.2)
        self.assertIs(node.core.grid, grid)
        self.assertEqual(grid.last_depth, 4.0)
        self.assertEqual(node.depth_rejected_reason, 'DEPTH_FRAME_INVALID')

    def test_worker_exception_preserves_map_and_newer_cache(self):
        """融合异常保留实时证据和更新缓存，诊断记录异常，不让旧任务清新帧。"""
        node, grid, _, errors = self.context(error=RuntimeError('受控融合异常'))
        latest = depth_message(5.1)
        node.pending_depth = latest
        FollowerNode.accept_depth_result(node, 5.2)
        self.assertIs(node.core.grid, grid)
        self.assertEqual(grid.last_depth, 4.0)
        self.assertIs(node.pending_depth, latest)
        self.assertIsNone(node.depth_future)
        self.assertEqual(node.depth_applied, 0)
        self.assertIn('受控融合异常', node.depth_worker_error)
        self.assertEqual(len(errors), 1)

    def test_version_rejection_resubmits_latest_on_private_current_map(self):
        """人工确认改变证据后拒收旧副本并重算；不能排第二任务或共享可写地图。"""
        node, grid, _, _ = self.context()
        grid.recenter(1.0, .4)
        self.assertTrue(grid.confirm_start(1.0, .4, 5.1))
        latest = node.pending_depth
        node.depth_info = SimpleNamespace(width=4, height=4, k=[2., 0., 2., 0., 2., 2., 0., 0., 1.])
        transform = SimpleNamespace(rotation=SimpleNamespace(x=0., y=0., z=0., w=1.),
                                    translation=SimpleNamespace(x=0., y=0., z=1.))
        node.tf_buffer = SimpleNamespace(lookup_transform=lambda *arguments: SimpleNamespace(transform=transform))
        node.depth_pool = DeferredPool()
        node.fuse_depth = FollowerNode.fuse_depth
        FollowerNode.integrate_depth(node, 5.2)
        self.assertEqual(node.depth_rejected_reason, 'DEPTH_MAP_CHANGED')
        self.assertIs(node.pending_depth, latest)
        self.assertEqual(len(node.depth_pool.calls), 1)
        _, arguments, future = node.depth_pool.calls[0]
        self.assertIsNot(arguments[0], grid)
        np.testing.assert_array_equal(arguments[0].origin, grid.origin)
        self.assertIs(node.depth_job['grid'], grid)
        self.assertEqual(node.depth_job['version'], grid.version)
        self.assertIs(node.depth_future, future)
        FollowerNode.integrate_depth(node, 5.3)
        self.assertEqual(len(node.depth_pool.calls), 1)
        self.assertIs(node.depth_future, future)


    def test_camera_model_uses_capture_pose_and_private_snapshot(self):
        """最新机身已移动时，模型仍须按图像采集位姿分离安装外参，且不提前修改当前地图。"""
        node, grid, _, _ = self.context()
        node.depth_future, node.depth_job = None, None
        node.core.history = PoseHistory()
        node.core.history.add(Pose(5., 1., 2., .4))
        node.core.history.add(Pose(5.2, 8., 9., 1.))
        node.depth_info = SimpleNamespace(width=4, height=4, k=[2., 0., 2., 0., 2., 2., 0., 0., 1.])
        transform = SimpleNamespace(rotation=SimpleNamespace(x=0., y=0., z=0., w=1.),
                                    translation=SimpleNamespace(x=1.2, y=2.1, z=.43))
        node.tf_buffer = SimpleNamespace(lookup_transform=lambda *arguments: SimpleNamespace(transform=transform))
        node.depth_pool = DeferredPool()
        node.fuse_depth = FollowerNode.fuse_depth
        FollowerNode.integrate_depth(node, 5.2)
        self.assertIsNone(grid.camera_model)
        snapshot = node.depth_pool.calls[0][1][0]
        rotation, translation = snapshot.camera_model.transform([1., 2.], .4)
        np.testing.assert_allclose(rotation, np.eye(3), atol=1e-12)
        np.testing.assert_allclose(translation, [1.2, 2.1, .43], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
