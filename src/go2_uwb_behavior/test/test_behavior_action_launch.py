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

"""验证跟随与随机漫游 Action、半径约束、互斥及计算门控."""

import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool

from go2_uwb_behavior.action import FollowUwb, RandomRoam
from go2_uwb_behavior.srv import SetBehavior


@pytest.mark.launch_test
def generate_test_description():
    """启动关闭实机速度且缩小采样半径的行为节点."""
    behavior_node = launch_ros.actions.Node(
        package="go2_uwb_behavior",
        executable="uwb_behavior_controller_node",
        name="uwb_behavior_controller_node",
        output="screen",
        parameters=[
            {
                "default_mode": "IDLE",
                "enable_motion": False,
                "uwb_median_window": 3,
                "minimum_owner_samples": 3,
                "owner_keepout_radius": 0.20,
                "random_goal_radius_max": 0.80,
                "random_step_min": 0.20,
                "random_step_max": 0.80,
                "goal_obstacle_clearance": 0.10,
                "roam_goal_tolerance": 0.10,
                "readiness_timeout_sec": 3.0,
                "input_recovery_timeout_sec": 2.0,
                "stop_confirm_sec": 0.10,
                "progress_window_sec": 10.0,
            }
        ],
    )
    return (
        launch.LaunchDescription(
            [behavior_node, launch_testing.actions.ReadyToTest()]
        ),
        {"behavior_node": behavior_node},
    )


class TestBehaviorAction(unittest.TestCase):
    """使用伪造传感输入验证节点级接口和状态转换."""

    @classmethod
    def setUpClass(cls):
        """初始化测试 ROS 客户端."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """关闭测试 ROS 上下文."""
        rclpy.shutdown()

    def setUp(self):
        """创建输入发布者、目标订阅者及 Action/Service 客户端."""
        self.node = rclpy.create_node("behavior_action_test_client")
        self.target_pub = self.node.create_publisher(
            PointStamped, "/uwb/target_point", 10
        )
        self.odom_pub = self.node.create_publisher(Odometry, "/leg_odom2", 10)
        self.obstacle_pub = self.node.create_publisher(
            PointCloud2, "/local_rolling_obstacle", 10
        )
        self.planner_cmd_pub = self.node.create_publisher(
            Twist, "/go2_uwb_behavior/planner_cmd_vel", 10
        )
        self.latest_target = None
        self.latest_compute_enable = None
        self.target_sub = self.node.create_subscription(
            PoseStamped,
            "/go2/random_roam/target",
            self._target_callback,
            10,
        )
        compute_qos = QoSProfile(depth=1)
        compute_qos.reliability = ReliabilityPolicy.RELIABLE
        compute_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.compute_enable_sub = self.node.create_subscription(
            Bool,
            "/go2_uwb_behavior/compute_enable",
            self._compute_enable_callback,
            compute_qos,
        )
        self.action_client = ActionClient(
            self.node, RandomRoam, "/go2/random_roam"
        )
        self.follow_action_client = ActionClient(
            self.node, FollowUwb, "/go2/follow_uwb"
        )
        self.behavior_client = self.node.create_client(
            SetBehavior, "/go2/set_behavior"
        )
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.publish_planner_command = True

    def tearDown(self):
        """销毁本用例创建的 ROS 节点."""
        self.node.destroy_node()

    def _target_callback(self, message):
        """保存行为节点发布的最新固定随机目标."""
        self.latest_target = message

    def _compute_enable_callback(self, message):
        """保存行为层发布的最新重计算门控状态."""
        self.latest_compute_enable = message.data

    def _publish_inputs(self):
        """发布主人位于 odom 原点时一致的 UWB、里程计和空障碍数据."""
        stamp = self.node.get_clock().now().to_msg()

        target = PointStamped()
        target.header.stamp = stamp
        target.header.frame_id = "base_footprint"
        target.point.x = -self.robot_x
        target.point.y = -self.robot_y
        self.target_pub.publish(target)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = "base_footprint"
        cloud.height = 1
        cloud.width = 0
        cloud.fields = [
            PointField(
                name="x", offset=0, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="y", offset=4, datatype=PointField.FLOAT32, count=1
            ),
        ]
        cloud.point_step = 8
        cloud.row_step = 0
        cloud.is_dense = True
        self.obstacle_pub.publish(cloud)
        if self.publish_planner_command:
            self.planner_cmd_pub.publish(Twist())

    def _pump_until(self, predicate, timeout_sec):
        """持续发送新鲜输入并处理回调，直到条件成立或超时."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self._publish_inputs()
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return True
            time.sleep(0.02)
        return False

    def _set_mode(self, mode):
        """调用行为服务并等待模式请求被节点接收."""
        request = SetBehavior.Request()
        request.mode = mode
        future = self.behavior_client.call_async(request)
        self.assertTrue(self._pump_until(future.done, 3.0))
        self.assertTrue(future.result().accepted)
        return future.result()

    def _send_goal(
        self, seed, timeout_sec=10.0, min_radius=0.0, max_radius=0.0
    ):
        """发送一次漫游目标并等待 Action Server 给出接收结果."""
        goal = RandomRoam.Goal()
        goal.random_seed = seed
        goal.timeout_sec = timeout_sec
        goal.min_radius = min_radius
        goal.max_radius = max_radius
        future = self.action_client.send_goal_async(goal)
        self.assertTrue(self._pump_until(future.done, 3.0))
        return future.result()

    def _send_follow_goal(self, timeout_sec=0.0):
        """发送持续跟随任务；timeout_sec 为 0 时由上层 cancel 结束."""
        goal = FollowUwb.Goal()
        goal.timeout_sec = timeout_sec
        future = self.follow_action_client.send_goal_async(goal)
        self.assertTrue(self._pump_until(future.done, 3.0))
        return future.result()

    def test_roam_complete_interface_lifecycle(self):
        """覆盖 STOP、到达、取消、超时、互斥和按需计算完整接口链路."""
        self.assertTrue(self.action_client.wait_for_server(timeout_sec=5.0))
        self.assertTrue(
            self.follow_action_client.wait_for_server(timeout_sec=5.0)
        )
        self.assertTrue(self.behavior_client.wait_for_service(timeout_sec=5.0))
        self.assertTrue(
            self._pump_until(
                lambda: self.node.count_publishers("/cmd_vel") == 1
                and self.node.count_publishers(
                    "/go2_uwb_local_follow/nominal_cmd"
                )
                == 1,
                3.0,
            )
        )
        self.assertTrue(
            self._pump_until(
                lambda: self.latest_compute_enable is False, 3.0
            )
        )

        stop_response = self._set_mode(SetBehavior.Request.STOP)
        self.assertEqual(stop_response.current_mode, SetBehavior.Request.STOP)
        rejected_handle = self._send_goal(1)
        self.assertFalse(rejected_handle.accepted)
        self._set_mode(SetBehavior.Request.IDLE)

        invalid_radius_handle = self._send_goal(
            2, min_radius=0.10, max_radius=0.80
        )
        self.assertFalse(invalid_radius_handle.accepted)

        first_handle = self._send_goal(42)
        self.assertTrue(first_handle.accepted)
        self.assertTrue(
            self._pump_until(
                lambda: self.latest_compute_enable is True, 3.0
            )
        )
        first_result_future = first_handle.get_result_async()
        self.assertTrue(
            self._pump_until(lambda: self.latest_target is not None, 3.0)
        )

        self.robot_x = self.latest_target.pose.position.x
        self.robot_y = self.latest_target.pose.position.y
        self.assertTrue(self._pump_until(first_result_future.done, 3.0))
        first_response = first_result_future.result()
        self.assertEqual(first_response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(first_response.result.code, RandomRoam.Result.SUCCESS)
        center = first_response.result.center_pose.pose.position
        target = first_response.result.target_pose.pose.position
        radius = ((target.x - center.x) ** 2 + (target.y - center.y) ** 2) ** 0.5
        self.assertGreaterEqual(radius, 0.20)
        self.assertLessEqual(radius, 0.80)
        self.assertTrue(
            self._pump_until(
                lambda: self.latest_compute_enable is False, 3.0
            )
        )

        self.latest_target = None
        cancel_handle = self._send_goal(9)
        self.assertTrue(cancel_handle.accepted)
        cancel_result_future = cancel_handle.get_result_async()
        cancel_future = cancel_handle.cancel_goal_async()
        self.assertTrue(self._pump_until(cancel_future.done, 3.0))
        self.assertEqual(len(cancel_future.result().goals_canceling), 1)
        self.assertTrue(self._pump_until(cancel_result_future.done, 3.0))
        cancel_response = cancel_result_future.result()
        self.assertEqual(cancel_response.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(cancel_response.result.code, RandomRoam.Result.CANCELED)

        # 测试使用瞬移模拟到达；先回到中心，避免跨目标瞬移被正确识别为里程计跳变。
        self.robot_x = 0.0
        self.robot_y = 0.0
        self._pump_until(lambda: False, 0.2)
        self.latest_target = None
        recovery_handle = self._send_goal(12)
        self.assertTrue(recovery_handle.accepted)
        recovery_result = recovery_handle.get_result_async()
        self.assertTrue(self._pump_until(lambda: self.latest_target is not None, 3.0))
        self.publish_planner_command = False
        self._pump_until(lambda: False, 0.5)
        self.assertFalse(recovery_result.done())
        self.publish_planner_command = True
        self._pump_until(lambda: False, 0.3)
        self.assertFalse(recovery_result.done())
        self.robot_x = self.latest_target.pose.position.x
        self.robot_y = self.latest_target.pose.position.y
        self.assertTrue(self._pump_until(recovery_result.done, 3.0))
        self.assertEqual(recovery_result.result().result.code, RandomRoam.Result.SUCCESS)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self._pump_until(lambda: False, 0.2)
        self.latest_target = None
        input_timeout_handle = self._send_goal(10)
        self.assertTrue(input_timeout_handle.accepted)
        input_timeout_result_future = input_timeout_handle.get_result_async()
        self.assertTrue(
            self._pump_until(lambda: self.latest_target is not None, 3.0)
        )
        self.publish_planner_command = False
        self.assertTrue(self._pump_until(input_timeout_result_future.done, 3.0))
        input_timeout_response = input_timeout_result_future.result()
        self.assertEqual(
            input_timeout_response.status, GoalStatus.STATUS_ABORTED
        )
        self.assertEqual(
            input_timeout_response.result.code,
            RandomRoam.Result.INPUT_TIMEOUT,
        )
        self.publish_planner_command = True

        timeout_handle = self._send_goal(11, timeout_sec=5.0)
        self.assertTrue(timeout_handle.accepted)
        timeout_result_future = timeout_handle.get_result_async()
        self.assertTrue(self._pump_until(timeout_result_future.done, 7.0))
        timeout_response = timeout_result_future.result()
        self.assertEqual(timeout_response.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(timeout_response.result.code, RandomRoam.Result.TIMEOUT)

        follow_handle = self._send_follow_goal()
        self.assertTrue(follow_handle.accepted)
        follow_result_future = follow_handle.get_result_async()
        rejected_during_follow = self._send_goal(6)
        self.assertFalse(rejected_during_follow.accepted)
        follow_cancel_future = follow_handle.cancel_goal_async()
        self.assertTrue(self._pump_until(follow_cancel_future.done, 3.0))
        self.assertTrue(self._pump_until(follow_result_future.done, 3.0))
        self.assertEqual(
            follow_result_future.result().status, GoalStatus.STATUS_CANCELED
        )
        self.assertEqual(
            follow_result_future.result().result.code,
            FollowUwb.Result.CANCELED,
        )

        self.latest_target = None
        second_handle = self._send_goal(7)
        self.assertTrue(second_handle.accepted)
        second_result_future = second_handle.get_result_async()

        self._set_mode(SetBehavior.Request.STOP)
        self.assertTrue(self._pump_until(second_result_future.done, 3.0))
        second_response = second_result_future.result()
        self.assertEqual(second_response.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(
            second_response.result.code,
            RandomRoam.Result.PREEMPTED_BY_MODE,
        )
        self._set_mode(SetBehavior.Request.IDLE)


@launch_testing.post_shutdown_test()
class TestBehaviorProcessExit(unittest.TestCase):
    """验证测试结束时行为节点可以被正常关闭."""

    def test_exit_code(self, proc_info, behavior_node):
        """要求 launch 关闭节点时没有异常退出码."""
        launch_testing.asserts.assertExitCodes(
            proc_info, process=behavior_node
        )
