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

"""隔离验证普通跟随与行为 FOLLOW 都能安全跨越大角度绕障门限."""

import functools
import math
import struct
import time
import unittest

from geometry_msgs.msg import PointStamped, Twist, TwistStamped
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
import rclpy
from sensor_msgs.msg import PointCloud2, PointField


@pytest.mark.launch_test
def generate_test_description():
    """分别启动独立跟随和行为 FOLLOW 的完整规划闭环，全部输出到测试话题."""
    processes = []
    for branch in ('follow', 'behavior'):
        prefix = '/heading_test/' + branch
        common = {
            'target_topic': '/heading_test/target',
            'odom_topic': '/heading_test/odom',
            'nominal_cmd_topic': prefix + '/nominal',
            'avoidance_feedback_topic': prefix + '/feedback',
            'heading_stop_angle': 1.05,
            'enable_avoidance_heading_relaxation': True,
        }
        if branch == 'follow':
            controller = launch_ros.actions.Node(
                package='go2_uwb_local_follow', executable='uwb_follow_controller_node',
                name='heading_test_follow', parameters=[common, {
                    'enable_motion': False,
                    'cmd_vel_topic': prefix + '/disabled_cmd',
                    'diagnostics_topic': prefix + '/follow_status',
                }])
        else:
            controller = launch_ros.actions.Node(
                package='go2_uwb_behavior', executable='uwb_behavior_controller_node',
                name='heading_test_behavior', parameters=[common, {
                    'enable_motion': True, 'default_mode': 'FOLLOW',
                    'obstacle_topic': '/heading_test/cloud',
                    'planner_cmd_topic': prefix + '/planner_cmd',
                    'cmd_vel_topic': prefix + '/cmd',
                    'planner_diagnostics_topic': prefix + '/planner_status',
                    'diagnostics_topic': prefix + '/behavior_status',
                    'follow_diagnostics_topic': prefix + '/follow_status',
                    'behavior_service_name': prefix + '/set_behavior',
                    'roam_action_name': prefix + '/roam',
                }])
        planner = launch_ros.actions.Node(
            package='go2_uwb_local_follow', executable='local_velocity_planner_node',
            name='heading_test_' + branch + '_planner', parameters=[{
                'enable_motion': True, 'enable_emergency_reverse': False,
                'enable_self_filter': False,
                'odom_topic': '/heading_test/odom',
                'obstacle_topic': '/heading_test/cloud',
                'nominal_cmd_topic': prefix + '/nominal',
                'avoidance_feedback_topic': prefix + '/feedback',
                'cmd_vel_topic': prefix + ('/cmd' if branch == 'follow' else '/planner_cmd'),
                'planned_cmd_topic': prefix + '/planned',
                'final_cmd_topic': prefix + '/final',
                'selected_path_topic': prefix + '/path',
                'diagnostics_topic': prefix + '/planner_status',
                'odom_timeout_sec': 0.2,
                'prediction_time': 1.5, 'robot_width': 0.38, 'safety_margin': 0.05,
                'max_linear_accel': 0.5, 'max_linear_decel': 0.7,
                'max_angular_speed': 1.5, 'max_angular_accel': 1.5,
                'obstacle_influence_distance': 0.5,
            }])
        processes.extend([controller, planner])
    return (launch.LaunchDescription(processes + [launch_testing.actions.ReadyToTest()]),
            {'processes': processes})


class TestHeadingAvoidance(unittest.TestCase):
    """同一套输入分别验证两条生产跟随链路的实际速度输出."""

    @classmethod
    def setUpClass(cls):
        """初始化隔离 ROS 上下文."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """释放 ROS 上下文."""
        rclpy.shutdown()

    def setUp(self):
        """创建目标与传感器输入，并记录两条链路的输出和安全绕障反馈."""
        self.node = rclpy.create_node('heading_avoidance_test_client')
        self.target_pub = self.node.create_publisher(PointStamped, '/heading_test/target', 10)
        self.odom_pub = self.node.create_publisher(Odometry, '/heading_test/odom', 10)
        self.cloud_pub = self.node.create_publisher(PointCloud2, '/heading_test/cloud', 10)
        self.commands = {branch: [] for branch in ('follow', 'behavior')}
        self.feedback = {branch: [] for branch in ('follow', 'behavior')}
        for branch in self.commands:
            self.node.create_subscription(
                Twist, '/heading_test/' + branch + '/cmd',
                functools.partial(self.record_command, branch), 10)
            self.node.create_subscription(
                TwistStamped, '/heading_test/' + branch + '/feedback',
                functools.partial(self.record_feedback, branch), 10)
        self.heading = 0.0
        self.distance = 3.0
        self.points = []
        self.send_target = True
        self.pump(0.7)

    def tearDown(self):
        """销毁测试接口."""
        self.node.destroy_node()

    def record_command(self, branch, message):
        """记录最终实际隔离输出的前进速度."""
        self.commands[branch].append(message.linear.x)

    def record_feedback(self, branch, message):
        """记录规划器是否仍在批准安全的前进绕障."""
        self.feedback[branch].append(message.twist.linear.x)

    def pump(self, duration):
        """供给新鲜输入，排空测试回调以避免把队列延迟误判成控制行为."""
        end = time.monotonic() + duration
        while time.monotonic() < end:
            stamp = self.node.get_clock().now().to_msg()
            if self.send_target:
                target = PointStamped()
                target.header.stamp = stamp
                target.header.frame_id = 'base_footprint'
                target.point.x = self.distance * math.cos(math.radians(self.heading))
                target.point.y = self.distance * math.sin(math.radians(self.heading))
                self.target_pub.publish(target)
            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(odom)
            cloud = PointCloud2()
            cloud.header.stamp = stamp
            cloud.header.frame_id = 'base_footprint'
            cloud.height = 1
            cloud.width = len(self.points)
            cloud.fields = [PointField(name=name, offset=index * 4,
                                       datatype=PointField.FLOAT32, count=1)
                            for index, name in enumerate(('x', 'y', 'z'))]
            cloud.point_step = 12
            cloud.row_step = 12 * cloud.width
            cloud.data = b''.join(struct.pack('<fff', *point) for point in self.points)
            self.cloud_pub.publish(cloud)
            rclpy.spin_once(self.node, timeout_sec=0.005)
            for _ in range(16):
                rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(0.015)

    def assert_forward(self, maximum=None):
        """两条链路的近期输出都应持续前进，可选检查低速上限."""
        for branch, values in self.commands.items():
            self.assertGreaterEqual(len(values), 3, branch)
            self.assertTrue(all(value > 0.0 for value in values[-3:]), (branch, values[-5:]))
            if maximum is not None:
                self.assertTrue(all(value <= maximum for value in values[-3:]), branch)

    def assert_stopped(self):
        """两条链路均应持续输出零线速度."""
        for branch, values in self.commands.items():
            self.assertGreaterEqual(len(values), 3, branch)
            self.assertTrue(all(value == 0.0 for value in values[-3:]), (branch, values[-5:]))

    def enter_safe_avoidance(self):
        """先在普通角度建立安全绕障反馈，再跨过原先的约 60 度门限."""
        self.points = [(0.0, -0.75, 0.2)]
        self.heading = 50.0
        self.pump(0.7)
        self.assert_forward()
        for branch, values in self.feedback.items():
            self.assertTrue(values and values[-1] > 0.0, (branch, values[-5:]))
        self.heading = 70.0
        self.pump(0.6)
        self.assert_forward(0.50)

    def test_clear_space_still_aligns_with_hysteresis(self):
        """无障碍时保持原对准策略，60 度附近抖动不会恢复前进."""
        self.heading = 50.0
        self.pump(0.4)
        self.assert_forward()
        self.heading = 62.0
        self.pump(0.4)
        self.assert_stopped()
        self.heading = 58.0
        self.pump(0.4)
        self.assert_stopped()
        self.heading = 50.0
        self.pump(0.4)
        self.assert_forward()

    def test_safe_bypass_respects_heading_and_distance_limits(self):
        """安全绕障跨越旧阈值不中断，超过扩展角度或已到跟随距离仍停车."""
        self.enter_safe_avoidance()
        self.heading = 86.0
        self.pump(0.4)
        self.assert_stopped()
        self.heading = 50.0
        self.pump(0.4)
        self.assert_forward()
        self.distance = 0.8
        self.heading = 70.0
        self.pump(0.4)
        self.assert_stopped()

    def test_collision_and_target_dropout_cancel_forward_motion(self):
        """即使已放宽角度，新障碍进入足迹或目标中断也必须停车."""
        self.enter_safe_avoidance()
        self.points.append((0.25, 0.0, 0.2))
        self.pump(0.4)
        self.assert_stopped()
        for branch, values in self.feedback.items():
            self.assertEqual(values[-1], 0.0, branch)
        self.heading = 0.0
        self.points = []
        self.pump(0.4)
        self.enter_safe_avoidance()
        self.send_target = False
        self.pump(0.7)
        self.assert_stopped()


@launch_testing.post_shutdown_test()
class TestHeadingProcessExit(unittest.TestCase):
    """检查新增反馈闭环的节点可以正常退出."""

    def test_exit_codes(self, proc_info, processes):
        """所有控制和规划节点都必须正常结束."""
        for process in processes:
            launch_testing.asserts.assertExitCodes(proc_info, process=process)
