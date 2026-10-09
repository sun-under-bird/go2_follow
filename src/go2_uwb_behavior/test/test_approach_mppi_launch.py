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

"""真实 MPPI 与行为节点的闭环软件测试；全部速度仅发往模拟话题."""
import math
import struct
import sys
import time
import unittest
from pathlib import Path

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import PointCloud2, PointField
from uwb_aoa_pkg.msg import UwbTarget

from go2_uwb_behavior.action import ApproachUwb


# 测试专用时钟不参与部署；采集时钟连续，接收和停止超时仍按真实时间判断。
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))
from ros_test_clock import MonotonicTestClock  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    """启动一套现有 MPPI 和唯一最终输出节点，保留碰撞及紧急制动保护."""
    follow_share = Path(get_package_share_directory("go2_uwb_local_follow"))
    behavior_share = Path(get_package_share_directory("go2_uwb_behavior"))
    # 真实 MPPI 只发布内部速度，不能连接实机底盘。
    planner = launch_ros.actions.Node(
        package="go2_uwb_local_follow", executable="local_velocity_planner_node",
        remappings=[("/clock", "/approach_mppi/clock")],
        name="local_velocity_planner_node", parameters=[
            str(follow_share / "config" / "local_velocity_planner.yaml"), {
                "use_sim_time": True,
                "enable_motion": True, "nominal_cmd_topic": "/approach_mppi/nominal",
                "odom_topic": "/approach_mppi/odom", "obstacle_topic": "/approach_mppi/cloud",
                "cmd_vel_topic": "/approach_mppi/planner",
                "diagnostics_topic": "/approach_mppi/planner_diagnostics",
                "odom_timeout_sec": 0.3, "obstacle_timeout_sec": 0.3,
                "obstacle_slowdown_after_sec": 0.15,
                "nominal_timeout_sec": 0.3, "diagnostic_frequency": 20.0,
            }], output="screen")
    # 唯一最终速度出口进入软件运动模型，使用现有实机控制参数。
    behavior = launch_ros.actions.Node(
        package="go2_uwb_behavior", executable="uwb_behavior_controller_node",
        remappings=[("/clock", "/approach_mppi/clock")],
        name="uwb_behavior_controller_node", parameters=[
            str(behavior_share / "config" / "behavior_controller.yaml"), {
                "use_sim_time": True,
                "enable_motion": True, "targets_topic": "/approach_mppi/targets",
                "odom_topic": "/approach_mppi/odom", "obstacle_topic": "/approach_mppi/cloud",
                "nominal_cmd_topic": "/approach_mppi/nominal",
                "planner_cmd_topic": "/approach_mppi/planner",
                "planner_diagnostics_topic": "/approach_mppi/planner_diagnostics",
                "cmd_vel_topic": "/approach_mppi/cmd",
                "approach_action_name": "/approach_mppi/approach",
                "target_timeout_sec": 0.3, "odom_timeout_sec": 0.3,
                "obstacle_timeout_sec": 0.3, "planner_cmd_timeout_sec": 0.3,
                "arrival_stable_sec": 0.2, "stop_confirm_sec": 0.15,
                "progress_window_sec": 2.0,
            }], output="screen")
    return launch.LaunchDescription([planner, behavior, launch_testing.actions.ReadyToTest()]), {
        "planner": planner, "behavior": behavior}


class TestMppiApproach(unittest.TestCase):
    """简单位姿积分用于验证软件链路，不代表真实机器人动力学或硬件验收."""

    @classmethod
    def setUpClass(cls):
        """初始化 ROS 上下文."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """结束 ROS 上下文."""
        rclpy.shutdown()

    def test_closed_loop_success_and_collision_blocking(self):
        """实际 MPPI 接近固定 ID2 并停稳；足迹内障碍时拒绝继续运动并返回受阻."""
        self.node = rclpy.create_node("approach_mppi_model")
        self.clock = MonotonicTestClock(self.node, "/approach_mppi/clock")
        self.targets = self.node.create_publisher(UwbTarget, "/approach_mppi/targets", 64)
        self.odom = self.node.create_publisher(Odometry, "/approach_mppi/odom", 10)
        self.cloud = self.node.create_publisher(PointCloud2, "/approach_mppi/cloud", 10)
        self.command = Twist()
        self.nonzero_speeds = []
        self.diagnostics = []
        self.node.create_subscription(
            DiagnosticArray, "/approach_mppi/planner_diagnostics",
            self.diagnostics.append, 10)
        self.node.create_subscription(Twist, "/approach_mppi/cmd", self.record, 10)
        self.client = ActionClient(self.node, ApproachUwb, "/approach_mppi/approach")
        self.x = self.y = self.yaw = 0.0
        self.world_target_x = 1.5
        self.collision = False
        self.last_time = time.monotonic()
        handle = None
        try:
            self.assertTrue(self.client.wait_for_server(timeout_sec=5.0))
            self.pump(0.3)
            goal = ApproachUwb.Goal(target_id=2, stop_distance_m=0.5, timeout_sec=15.0)
            handle = self.wait(self.client.send_goal_async(goal), 3.0)
            self.assertTrue(handle.accepted)
            result = self.wait(handle.get_result_async(), 17.5)
            self.assertEqual((result.status, result.result.code),
                             (GoalStatus.STATUS_SUCCEEDED, ApproachUwb.Result.SUCCESS),
                             f"{result.result.message}; 最新规划器诊断={self.diagnostics[-1:]}")
            self.assertGreaterEqual(result.result.final_distance, 0.5)
            self.assertLessEqual(result.result.final_distance, 0.6)
            self.assertTrue(self.nonzero_speeds)
            self.assertGreaterEqual(min(self.nonzero_speeds), 0.25 - 1e-6)
            self.pump(0.2)
            self.assertEqual((self.command.linear.x, self.command.angular.z), (0.0, 0.0))
            self.world_target_x = self.x + 1.0
            self.collision = True
            handle = self.wait(self.client.send_goal_async(goal), 3.0)
            self.assertTrue(handle.accepted)
            result = self.wait(handle.get_result_async(), 5.0)
            self.assertEqual(result.result.code, ApproachUwb.Result.BLOCKED)
            self.assertEqual((self.command.linear.x, self.command.angular.z), (0.0, 0.0))
        finally:
            if handle and handle.status in (
                    GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                handle.cancel_goal_async()
                self.pump(0.8)
            self.client.destroy()
            self.node.destroy_node()

    def record(self, command):
        """保存唯一最终速度，记录非零线速度以确认未生成不可执行的小步速."""
        self.command = command
        if command.linear.x > 1e-6:
            self.nonzero_speeds.append(command.linear.x)

    def feed(self):
        """按收到的最终速度积分平面位姿，并生成一致的标签、里程计和障碍采集."""
        current = time.monotonic()
        dt = min(current - self.last_time, 0.05)
        self.last_time = current
        self.x += self.command.linear.x * math.cos(self.yaw) * dt
        self.y += self.command.linear.x * math.sin(self.yaw) * dt
        self.yaw += self.command.angular.z * dt
        stamp = self.clock.sample()
        target = UwbTarget()
        target.header.stamp, target.header.frame_id = stamp, "base_footprint"
        target.target_id, target.valid, target.confidence = 2, True, 100
        dx, dy = self.world_target_x - self.x, -self.y
        target.position.x = dx * math.cos(self.yaw) + dy * math.sin(self.yaw)
        target.position.y = -dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
        self.targets.publish(target)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = (
            math.sin(self.yaw / 2), math.cos(self.yaw / 2))
        odom.twist.twist = self.command
        self.odom.publish(odom)
        cloud = PointCloud2()
        cloud.header.stamp, cloud.header.frame_id = stamp, "base_footprint"
        cloud.height, cloud.width, cloud.point_step = 1, int(self.collision), 12
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)]
        cloud.row_step = cloud.width * cloud.point_step
        cloud.data = struct.pack("<fff", 0.1, 0.0, 0.2) if self.collision else bytes()
        self.cloud.publish(cloud)

    def pump(self, seconds):
        """以实时采集维持闭环，不通过重复旧时间戳掩盖断流."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.feed()
            rclpy.spin_once(self.node, timeout_sec=0.01)
            time.sleep(0.01)

    def wait(self, future, timeout):
        """等待 Action 响应同时运行模拟底盘."""
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            self.pump(0.025)
        self.assertTrue(future.done(), "真实 MPPI 闭环等待超时")
        return future.result()


@launch_testing.post_shutdown_test()
class TestExit(unittest.TestCase):
    """确认真实规划器及行为节点退出."""

    def test_exit(self, proc_info, planner, behavior):
        """两节点应正常响应清理信号."""
        launch_testing.asserts.assertExitCodes(proc_info, process=planner)
        launch_testing.asserts.assertExitCodes(proc_info, process=behavior)
