# Copyright 2026 OpenAI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""使用隔离速度话题验证接近状态机、异常与四种 Action 互斥."""
import math
import sys
import time
import unittest
from pathlib import Path

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PointStamped, Twist, TwistStamped
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool
from uwb_aoa_pkg.msg import UwbTarget

from go2_uwb_behavior.action import ApproachUwb, FollowUwb, OrbitUwbOnce, RandomRoam
from go2_uwb_behavior.srv import SetBehavior


# 测试专用时钟不参与部署；采集时钟连续，接收和停止超时仍按真实时间判断。
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))
from ros_test_clock import MonotonicTestClock  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    """先覆盖禁运动节点，再在专用假底盘话题检查实际门控，不连接真实 /cmd_vel."""
    shared = {
        "use_sim_time": True,
        "default_mode": "IDLE", "publish_idle_velocity": False,
        "targets_topic": "/approach_test/targets", "target_topic": "/approach_test/person",
        "odom_topic": "/approach_test/odom", "obstacle_topic": "/approach_test/cloud",
        "planner_cmd_topic": "/approach_test/planner",
        "planner_diagnostics_topic": "/approach_test/planner_diagnostics",
        "target_timeout_sec": 0.15, "odom_timeout_sec": 0.15,
        "obstacle_timeout_sec": 0.15, "planner_cmd_timeout_sec": 0.15,
        "readiness_timeout_sec": 0.6, "input_recovery_timeout_sec": 0.3,
        "arrival_stable_sec": 0.18, "stop_confirm_sec": 0.12,
        "stop_confirmation_timeout_sec": 0.45, "feedback_frequency": 40.0,
        "diagnostic_frequency": 20.0,
        "planner_blocked_timeout_sec": 0.20, "progress_window_sec": 0.9,
        "uwb_median_window": 1, "minimum_owner_samples": 1,
    }
    nodes = []
    for prefix, motion, only in (
            ("approach_dry", False, False), ("approach_test", True, False),
            ("approach_only", False, True)):
        parameters = dict(shared)
        parameters.update({
            "enable_motion": motion, "target_only": only,
            "nominal_cmd_topic": f"/{prefix}/nominal", "cmd_vel_topic": f"/{prefix}/cmd",
            "compute_enable_topic": f"/{prefix}/compute",
            "diagnostics_topic": f"/{prefix}/diagnostics",
            "follow_diagnostics_topic": f"/{prefix}/follow_diagnostics",
            "behavior_service_name": f"/{prefix}/set_behavior",
            "approach_action_name": f"/{prefix}/approach",
            "follow_action_name": f"/{prefix}/follow", "orbit_action_name": f"/{prefix}/orbit",
            "roam_action_name": f"/{prefix}/roam",
        })
        # 每个测试节点使用独立 Action 和速度出口；纯目标节点不创建速度发布者。
        nodes.append(launch_ros.actions.Node(
            package="go2_uwb_behavior", executable="uwb_behavior_controller_node",
            remappings=[("/clock", "/approach_test/clock")],
            name=prefix, parameters=[parameters], output="screen"))
    return launch.LaunchDescription(nodes + [launch_testing.actions.ReadyToTest()]), {
        "behavior_nodes": nodes}


class TestApproach(unittest.TestCase):
    """用真实 ROS Action 检查安全结果；断流和停止仍按实际经过时间检查."""

    @classmethod
    def setUpClass(cls):
        """启动客户端上下文."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """结束上下文."""
        rclpy.shutdown()

    def setUp(self):
        """重置模拟输入、建立客户端，并显式解除上个用例可能留下的 STOP."""
        self.node = rclpy.create_node("approach_test_client")
        self.clock = MonotonicTestClock(self.node, "/approach_test/clock")
        self.targets = {1: (2.0, -1.0), 2: (2.0, 0.0)}
        self.tag_enabled = {1: True, 2: True}
        self.target_valid = True
        self.odom_enabled = self.cloud_enabled = self.planner_enabled = True
        self.robot_x = self.actual_v = 0.0
        self.move_pose = False
        self.frozen_odom_stamp = None
        self.planner_state = "TRACKING"
        self.planner_override = None
        self.nominal = Twist()
        self.outputs, self.dry_outputs, self.feedback = [], [], []
        self.behavior_diagnostics = []
        self.compute = None
        self.handles = []
        self.target_pub = self.node.create_publisher(UwbTarget, "/approach_test/targets", 64)
        self.person_pub = self.node.create_publisher(PointStamped, "/approach_test/person", 10)
        self.odom_pub = self.node.create_publisher(Odometry, "/approach_test/odom", 10)
        self.cloud_pub = self.node.create_publisher(PointCloud2, "/approach_test/cloud", 10)
        self.planner_pub = self.node.create_publisher(Twist, "/approach_test/planner", 10)
        self.diagnostics_pub = self.node.create_publisher(
            DiagnosticArray, "/approach_test/planner_diagnostics", 10)
        self.node.create_subscription(
            TwistStamped, "/approach_test/nominal", self.record_nominal, 10)
        self.node.create_subscription(Twist, "/approach_test/cmd", self.outputs.append, 10)
        self.node.create_subscription(Twist, "/approach_dry/cmd", self.dry_outputs.append, 10)
        self.node.create_subscription(
            DiagnosticArray, "/approach_test/diagnostics",
            self.behavior_diagnostics.append, 20)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Bool, "/approach_test/compute", self.record_compute, qos)
        self.approach = ActionClient(self.node, ApproachUwb, "/approach_test/approach")
        self.dry = ActionClient(self.node, ApproachUwb, "/approach_dry/approach")
        self.only = ActionClient(self.node, ApproachUwb, "/approach_only/approach")
        self.follow = ActionClient(self.node, FollowUwb, "/approach_test/follow")
        self.orbit = ActionClient(self.node, OrbitUwbOnce, "/approach_test/orbit")
        self.roam = ActionClient(self.node, RandomRoam, "/approach_test/roam")
        self.service = self.node.create_client(SetBehavior, "/approach_test/set_behavior")
        for client in (self.approach, self.dry, self.only, self.follow, self.orbit, self.roam):
            self.assertTrue(client.wait_for_server(timeout_sec=5.0))
        self.assertTrue(self.service.wait_for_service(timeout_sec=5.0))
        self.pump(0.20)
        self.set_mode(SetBehavior.Request.IDLE)

    def tearDown(self):
        """用例失败也发送取消并恢复输入，再销毁客户端；launch 框架回收节点进程."""
        self.odom_enabled = self.cloud_enabled = self.planner_enabled = True
        self.frozen_odom_stamp = None
        self.actual_v = 0.0
        self.move_pose = False
        for handle in self.handles:
            if handle.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                handle.cancel_goal_async()
        self.pump(0.65)
        for client in (self.approach, self.dry, self.only, self.follow, self.orbit, self.roam):
            client.destroy()
        self.node.destroy_node()

    def record_nominal(self, message):
        """记录本任务名义速度，假规划器只回传该速度而不另建控制器."""
        self.nominal = message.twist

    def record_compute(self, message):
        """保存瞬态感知门控."""
        self.compute = message.data

    def feed(self):
        """分别开关每种输入，用连续采集时间测试过期；模拟速度仅发往专用话题."""
        stamp = self.clock.sample()
        for tag, position in self.targets.items():
            if self.tag_enabled[tag]:
                target = UwbTarget()
                target.header.stamp, target.header.frame_id = stamp, "base_footprint"
                target.target_id, target.position.x, target.position.y = tag, *position
                target.confidence, target.valid = 100, self.target_valid
                self.target_pub.publish(target)
        person = PointStamped()
        person.header.stamp, person.header.frame_id = stamp, "base_footprint"
        person.point.x, person.point.y = self.targets[1]
        self.person_pub.publish(person)
        if self.odom_enabled:
            if self.move_pose:
                self.robot_x += 0.008
            odom = Odometry()
            odom.header.stamp = self.frozen_odom_stamp or stamp
            odom.header.frame_id, odom.child_frame_id = "odom", "base_footprint"
            odom.pose.pose.position.x = self.robot_x
            odom.pose.pose.orientation.w = 1.0
            odom.twist.twist.linear.x = self.actual_v
            self.odom_pub.publish(odom)
        if self.cloud_enabled:
            cloud = PointCloud2()
            cloud.header.stamp, cloud.header.frame_id = stamp, "base_footprint"
            cloud.height, cloud.width, cloud.point_step = 1, 0, 8
            cloud.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1)]
            self.cloud_pub.publish(cloud)
        if self.planner_enabled:
            self.planner_pub.publish(self.planner_override or self.nominal)
        status = DiagnosticStatus()
        status.hardware_id, status.message = "go2_stereo_local_planner", self.planner_state
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp, diagnostics.status = stamp, [status]
        self.diagnostics_pub.publish(diagnostics)

    def pump(self, seconds):
        """继续所有未禁用的传感输入，并处理客户端订阅."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.feed()
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.008)

    def wait(self, future, timeout=3.0):
        """等待响应时保持输入新鲜；超时给出最后反馈便于追根因."""
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self.pump(0.025)
        self.assertTrue(future.done(), f"等待超时，最近反馈={self.feedback[-5:]}")
        result = future.result()
        if hasattr(result, "result") and result.result.code == ApproachUwb.Result.STOP_UNCONFIRMED:
            evidence = [
                (status.message, {value.key: value.value for value in status.values})
                for message in self.behavior_diagnostics[-8:] for status in message.status]
            print(f"停车确认失败证据: {evidence}", flush=True)
        return result

    def set_mode(self, mode):
        """显式模式切换，STOP 只由后续明确 IDLE 解除."""
        request = SetBehavior.Request()
        request.mode = mode
        response = self.wait(self.service.call_async(request))
        self.assertTrue(response.accepted)
        return response

    def send(self, tag=2, distance=0.5, timeout=5.0, client=None):
        """发送可指定 ID/距离的接近请求，并收集状态反馈."""
        goal = ApproachUwb.Goal(target_id=tag, stop_distance_m=distance, timeout_sec=timeout)
        handle = self.wait((client or self.approach).send_goal_async(
            goal, feedback_callback=lambda message: self.feedback.append(message.feedback)))
        if handle.accepted:
            self.handles.append(handle)
        return handle

    def cancel(self, handle):
        """取消只能在停车结果后完成交接."""
        response = self.wait(handle.cancel_goal_async())
        self.assertEqual(len(response.goals_canceling), 1)
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(result.result.code, ApproachUwb.Result.CANCELED)

    def assert_zero(self):
        """检查最近安全门控输出已归零."""
        self.assertTrue(self.outputs)
        for command in self.outputs[-3:]:
            self.assertEqual((command.linear.x, command.angular.z), (0.0, 0.0))

    def test_00_disabled_and_target_only(self):
        """先验证 enable_motion=false 与 target_only 运动隔离."""
        self.assertFalse(self.send(client=self.only).accepted)
        handle = self.send(client=self.dry)
        self.pump(0.2)
        self.planner_override = Twist()
        self.planner_override.linear.x = 0.5
        self.pump(0.15)
        self.assertTrue(self.dry_outputs)
        self.assertTrue(all(c.linear.x == c.angular.z == 0.0 for c in self.dry_outputs))
        self.targets[2] = (0.55, 0.0)
        self.planner_override = None
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.result.code, ApproachUwb.Result.SUCCESS)
        self.assertTrue(self.dry_outputs)
        self.assertTrue(all(c.linear.x == c.angular.z == 0.0 for c in self.dry_outputs))
        self.assertEqual(self.node.count_publishers("/approach_only/cmd"), 0)
        self.assertEqual(self.node.count_publishers("/approach_test/cmd"), 1)

    def test_arrival_and_return_use_selected_id(self):
        """A/B/D：未到达不成功；稳定并停稳后结束，再用不同距离返回人员 ID1."""
        handle = self.send()
        pending = handle.get_result_async()
        self.pump(0.25)
        self.assertFalse(pending.done())
        self.assertTrue(any(c.linear.x > 0 for c in self.outputs))
        self.assertAlmostEqual(self.feedback[-1].current_distance, 2.0)
        self.assertAlmostEqual(self.feedback[-1].heading_error, 0.0)
        self.targets[2] = (0.55, 0.0)
        self.pump(0.08)
        self.assertFalse(pending.done())
        result = self.wait(pending)
        self.assertEqual((result.status, result.result.code),
                         (GoalStatus.STATUS_SUCCEEDED, ApproachUwb.Result.SUCCESS))
        self.assertEqual(result.result.target_id, 2)
        self.assertAlmostEqual(result.result.final_distance, 0.55)
        self.pump(0.06)
        self.assert_zero()
        self.assertFalse(self.compute)
        # ID2 仍在更新；下一任务只能读取 ID1，并使用新 Goal 的 1 米距离。
        self.targets[1] = (1.05, 0.0)
        second = self.send(tag=1, distance=1.0)
        result = self.wait(second.get_result_async())
        self.assertEqual(result.result.code, ApproachUwb.Result.SUCCESS)
        self.assertEqual(result.result.target_id, 1)
        self.assertAlmostEqual(result.result.final_distance, 1.05)

    def test_current_id_dropout_and_recovery(self):
        """C：ID1 更新不能掩盖 ID2 断流；恢复窗口内可恢复，持续丢失安全结束."""
        handle = self.send()
        self.pump(0.18)
        self.tag_enabled[2] = False
        self.pump(0.24)
        self.assertEqual(self.feedback[-1].state, ApproachUwb.Feedback.INPUT_PAUSED)
        self.assert_zero()
        self.tag_enabled[2] = True
        self.pump(0.18)
        self.assertEqual(self.feedback[-1].state, ApproachUwb.Feedback.APPROACHING)
        self.tag_enabled[2] = False
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.result.code, ApproachUwb.Result.TARGET_LOST)
        self.assert_zero()

    def test_mutual_exclusion_and_legacy_actions(self):
        """E：接近与三个原 Action 双向互斥，原有取消和停车链仍有效."""
        handle = self.send()
        for client, goal in (
                (self.approach, ApproachUwb.Goal(target_id=1, stop_distance_m=1.0)),
                (self.follow, FollowUwb.Goal()),
                (self.orbit, OrbitUwbOnce.Goal()),
                (self.roam, RandomRoam.Goal())):
            rejected = self.wait(client.send_goal_async(goal))
            self.assertFalse(rejected.accepted)
        self.cancel(handle)
        for client, goal in (
                (self.follow, FollowUwb.Goal()),
                (self.orbit, OrbitUwbOnce.Goal()),
                (self.roam, RandomRoam.Goal())):
            old = self.wait(client.send_goal_async(goal))
            self.assertTrue(old.accepted)
            self.handles.append(old)
            self.assertFalse(self.send().accepted)
            self.wait(old.cancel_goal_async())
            result = self.wait(old.get_result_async())
            self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)

    def test_stop_during_final_stop_latches(self):
        """F：到达停车中收到 STOP 仍抢占；不能自动开始 ID1 或被取消解除锁存."""
        self.targets[2] = (0.55, 0.0)
        self.actual_v = 0.12
        handle = self.send()
        deadline = time.monotonic() + 1.0
        while not any(f.state == ApproachUwb.Feedback.STOPPING for f in self.feedback):
            self.pump(0.025)
            self.assertLess(time.monotonic(), deadline)
        self.set_mode(SetBehavior.Request.STOP)
        self.wait(handle.cancel_goal_async())
        self.actual_v = 0.0
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(result.result.code, ApproachUwb.Result.PREEMPTED)
        self.assertFalse(self.send(tag=1, distance=1.0).accepted)
        self.assert_zero()
        self.set_mode(SetBehavior.Request.IDLE)
        self.targets[1] = (1.05, 0.0)
        next_handle = self.send(tag=1, distance=1.0)
        self.assertEqual(self.wait(next_handle.get_result_async()).result.code,
                         ApproachUwb.Result.SUCCESS)

    def test_moving_robot_and_final_distance_recheck(self):
        """距离达标但尚在运动不能成功；停车中标签离开后必须重新接近."""
        self.targets[2] = (0.55, 0.0)
        self.actual_v = 0.12
        handle = self.send()
        pending = handle.get_result_async()
        self.pump(0.38)
        self.assertFalse(pending.done())
        self.targets[2] = (0.9, 0.0)
        self.actual_v = 0.0
        self.pump(0.22)
        self.assertFalse(pending.done())
        self.targets[2] = (0.55, 0.0)
        self.assertEqual(self.wait(pending).result.code, ApproachUwb.Result.SUCCESS)

    def test_unconfirmed_stop_and_frozen_odom(self):
        """G：冻结位姿配非零速度及停更里程计都不能给出成功或解除 STOP."""
        for frozen in (False, True):
            self.set_mode(SetBehavior.Request.IDLE)
            self.targets[2] = (0.55, 0.0)
            self.actual_v = 0.12
            self.frozen_odom_stamp = None
            handle = self.send()
            self.pump(0.15)
            if frozen:
                self.frozen_odom_stamp = self.clock.sample()
            result = self.wait(handle.get_result_async())
            self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
            self.assertEqual(result.result.code, ApproachUwb.Result.STOP_UNCONFIRMED)
            self.assertFalse(self.send().accepted)
            self.assert_zero()
            self.actual_v = 0.0
            self.frozen_odom_stamp = None
            self.pump(0.2)

    def test_faults_timeout_blocking_and_unsafe_distance(self):
        """区分输入故障、规划受阻、超时和实际不安全间距，所有结束都发零."""
        self.assertFalse(self.send(distance=0.1).accepted)
        self.assertFalse(self.send(timeout=math.nan).accepted)
        for fault, expected in (
                ("cloud", ApproachUwb.Result.INPUT_TIMEOUT),
                ("blocked", ApproachUwb.Result.BLOCKED),
                ("timeout", ApproachUwb.Result.TIMEOUT),
                ("unsafe", ApproachUwb.Result.UNSAFE_DISTANCE),
                ("no_progress", ApproachUwb.Result.BLOCKED)):
            self.set_mode(SetBehavior.Request.IDLE)
            self.targets[2] = (2.0, 0.0)
            handle = self.send(timeout=0.4 if fault == "timeout" else 5.0)
            self.pump(0.16)
            if fault == "cloud":
                self.cloud_enabled = False
            elif fault == "blocked":
                self.planner_state = "BLOCKED"
            elif fault == "unsafe":
                self.targets[2] = (0.4, 0.0)
            result = self.wait(handle.get_result_async())
            self.assertEqual(result.result.code, expected, fault)
            self.assert_zero()
            self.cloud_enabled = True
            self.planner_state = "TRACKING"
            self.pump(0.10)

    def test_cancel_requires_stop_even_when_unconfirmed(self):
        """取消后不能以 CANCELED 业务码掩盖停车失败；先归零再返回 STOP_UNCONFIRMED."""
        handle = self.send()
        self.pump(0.18)
        self.actual_v = 0.12
        self.wait(handle.cancel_goal_async())
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(result.result.code, ApproachUwb.Result.STOP_UNCONFIRMED)
        self.assertFalse(self.send().accepted)
        self.assert_zero()

    def test_not_ready_and_final_stop_input_fault(self):
        """没有指定标签时 NOT_READY；停车阶段点云中断也不能误报成功."""
        self.tag_enabled[2] = False
        handle = self.send()
        self.assertEqual(self.wait(handle.get_result_async()).result.code,
                         ApproachUwb.Result.NOT_READY)
        self.tag_enabled[2] = True
        self.targets[2] = (0.55, 0.0)
        self.actual_v = 0.12
        self.feedback.clear()
        handle = self.send()
        deadline = time.monotonic() + 1.0
        while not any(f.state == ApproachUwb.Feedback.STOPPING for f in self.feedback):
            self.pump(0.025)
            self.assertLess(time.monotonic(), deadline)
        self.cloud_enabled = False
        self.pump(0.18)
        self.actual_v = 0.0
        result = self.wait(handle.get_result_async())
        self.assertEqual(result.result.code, ApproachUwb.Result.INPUT_TIMEOUT)

    def test_moving_person_does_not_require_distance_to_decrease(self):
        """人员等速移动时相对距离不下降，实际里程计前进仍是有效进展."""
        self.targets[1] = (2.0, 0.0)
        self.move_pose = True
        self.actual_v = 0.25
        handle = self.send(tag=1, distance=1.0)
        pending = handle.get_result_async()
        self.pump(1.1)
        self.assertFalse(pending.done())
        self.move_pose = False
        self.actual_v = 0.0
        self.cancel(handle)


@launch_testing.post_shutdown_test()
class TestExit(unittest.TestCase):
    """确认启动框架已回收三个测试进程."""

    def test_exit(self, proc_info, behavior_nodes):
        """所有测试节点应正常退出."""
        for node in behavior_nodes:
            launch_testing.asserts.assertExitCodes(proc_info, process=node)
