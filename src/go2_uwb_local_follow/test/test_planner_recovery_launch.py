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

"""在专用话题上验证规划器时效、运动补偿、断流恢复和地图延迟配对."""

import struct
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist, TwistStamped
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from uwb_aoa_pkg.msg import LibAoaRobotMsg


@pytest.mark.launch_test
def generate_test_description():
    """启动独立话题的规划器和滚动地图，实机速度话题完全隔离."""
    planner = launch_ros.actions.Node(
        package='go2_uwb_local_follow', executable='local_velocity_planner_node',
        name='test_recovery_planner', parameters=[{
            'enable_motion': True,
            'enable_emergency_reverse': False,
            'enable_self_filter': True,
            'nominal_cmd_topic': '/planner_test/nominal',
            'odom_topic': '/planner_test/odom',
            'obstacle_topic': '/planner_test/cloud',
            'cmd_vel_topic': '/planner_test/cmd',
            'planned_cmd_topic': '/planner_test/planned',
            'final_cmd_topic': '/planner_test/final',
            'selected_path_topic': '/planner_test/path',
            'diagnostics_topic': '/planner_test/status',
            'diagnostic_frequency': 20.0,
            'obstacle_timeout_sec': 0.7,
            'odom_timeout_sec': 0.2,
        }])
    rolling_map = launch_ros.actions.Node(
        package='go2_uwb_local_follow', executable='rolling_obstacle_map_node',
        name='test_recovery_map', parameters=[{
            'input_observation_topic': '/map_test/observation',
            'output_obstacle_topic': '/map_test/cloud',
            'odom_topic': '/map_test/odom',
            'diagnostics_topic': '/map_test/status',
            'odom_timeout_sec': 0.2,
        }])
    adapter = launch_ros.actions.Node(
        package='go2_uwb_local_follow', executable='uwb_target_adapter_node',
        name='test_recovery_adapter', parameters=[{
            'raw_topic': '/follow_test/raw', 'target_topic': '/follow_test/target',
            'diagnostics_topic': '/follow_test/adapter_status',
        }])
    follow = launch_ros.actions.Node(
        package='go2_uwb_local_follow', executable='uwb_follow_controller_node',
        name='test_recovery_follow', parameters=[{
            'enable_motion': False,
            'target_topic': '/follow_test/target', 'odom_topic': '/planner_test/odom',
            'nominal_cmd_topic': '/follow_test/nominal',
            'cmd_vel_topic': '/follow_test/disabled_cmd',
            'diagnostics_topic': '/follow_test/status',
        }])
    reverse = launch_ros.actions.Node(
        package='go2_uwb_local_follow', executable='local_velocity_planner_node',
        name='test_recovery_reverse', parameters=[{
            'enable_motion': True, 'enable_self_filter': False,
            'nominal_cmd_topic': '/planner_test/nominal',
            'odom_topic': '/planner_test/odom',
            'obstacle_topic': '/reverse_test/cloud',
            'cmd_vel_topic': '/reverse_test/cmd',
            'planned_cmd_topic': '/reverse_test/planned',
            'final_cmd_topic': '/reverse_test/final',
            'selected_path_topic': '/reverse_test/path',
            'diagnostics_topic': '/reverse_test/status',
            'emergency_reverse_distance': 0.4,
            'obstacle_timeout_sec': 0.7, 'odom_timeout_sec': 0.2,
        }])
    return (launch.LaunchDescription([
        planner, rolling_map, adapter, follow, reverse,
        launch_testing.actions.ReadyToTest()]),
        {'planner': planner, 'rolling_map': rolling_map,
         'adapter': adapter, 'follow': follow, 'reverse': reverse})


class TestPlannerRecovery(unittest.TestCase):
    """用可控时间戳和可开关输入复现控制链路的恢复边界."""

    @classmethod
    def setUpClass(cls):
        """初始化测试 ROS 上下文."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """关闭测试 ROS 上下文."""
        rclpy.shutdown()

    def setUp(self):
        """创建专用输入与观测接口."""
        self.node = rclpy.create_node('planner_recovery_client')
        self.nominal_pub = self.node.create_publisher(TwistStamped, '/planner_test/nominal', 10)
        self.odom_pub = self.node.create_publisher(Odometry, '/planner_test/odom', 10)
        self.cloud_pub = self.node.create_publisher(PointCloud2, '/planner_test/cloud', 10)
        self.map_odom_pub = self.node.create_publisher(Odometry, '/map_test/odom', 10)
        self.map_input_pub = self.node.create_publisher(PointCloud2, '/map_test/observation', 10)
        self.raw_pub = self.node.create_publisher(LibAoaRobotMsg, '/follow_test/raw', 10)
        self.reverse_pub = self.node.create_publisher(PointCloud2, '/reverse_test/cloud', 10)
        self.follow_commands = []
        self.reverse_commands = []
        self.send_raw = False
        self.raw_age_sec = 0
        self.reverse_points = None
        self.node.create_subscription(
            TwistStamped, '/follow_test/nominal', self.record_follow, 10)
        self.node.create_subscription(Twist, '/reverse_test/cmd', self.record_reverse, 10)
        self.commands = []
        self.states = []
        self.map_stamps = []
        self.node.create_subscription(Twist, '/planner_test/cmd', self.record_command, 10)
        self.node.create_subscription(
            DiagnosticArray, '/planner_test/status', self.record_state, 10)
        self.node.create_subscription(
            PointCloud2, '/map_test/cloud', self.record_map, qos_profile_sensor_data)
        self.send_odom = self.send_cloud = self.send_nominal = True
        self.fixed_stamp = None
        self.robot_x = 0.0
        self.point = (2.0, 2.0, 0.2)

    def tearDown(self):
        """销毁测试接口."""
        self.node.destroy_node()

    def record_follow(self, message):
        """记录实际适配器与跟随器链路生成的名义速度."""
        self.follow_commands.append(message.twist.linear.x)

    def record_reverse(self, message):
        """记录倒退恢复节点的真实隔离输出."""
        self.reverse_commands.append(message.linear.x)

    def record_command(self, message):
        """保存实际输出的前进速度."""
        self.commands.append(message.linear.x)

    def record_state(self, message):
        """保存诊断状态供几何补偿断言使用."""
        self.states.extend(status.message for status in message.status)

    def record_map(self, message):
        """保存地图源时间戳，确认延后处理没有给旧帧重新盖章."""
        self.map_stamps.append(message.header.stamp)

    def cloud(self, stamp):
        """生成符合约定的连续 FLOAT32 点云."""
        message = PointCloud2()
        message.header.stamp = stamp
        message.header.frame_id = 'base_footprint'
        message.height = message.width = 1
        message.fields = [PointField(name=name, offset=index * 4,
                                     datatype=PointField.FLOAT32, count=1)
                          for index, name in enumerate(('x', 'y', 'z'))]
        message.point_step = message.row_step = 12
        message.data = struct.pack('<fff', *self.point)
        return message

    def odom(self, stamp):
        """生成位姿与坐标系均合法的里程计."""
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_footprint'
        message.pose.pose.orientation.w = 1.0
        message.pose.pose.position.x = self.robot_x
        return message

    def pump(self, duration):
        """按输入开关持续供数并驱动测试订阅."""
        end = time.monotonic() + duration
        while time.monotonic() < end:
            stamp = self.node.get_clock().now().to_msg()
            if self.send_nominal:
                nominal = TwistStamped()
                nominal.header.stamp = stamp
                nominal.header.frame_id = 'base_footprint'
                nominal.twist.linear.x = 0.5
                self.nominal_pub.publish(nominal)
            if self.send_odom:
                self.odom_pub.publish(self.odom(stamp))
            if self.send_cloud:
                self.cloud_pub.publish(self.cloud(self.fixed_stamp or stamp))
            if self.send_raw:
                raw = LibAoaRobotMsg()
                raw.header.stamp = self.node.get_clock().now().to_msg()
                raw.header.stamp.sec -= self.raw_age_sec
                raw.header.frame_id = 'uwb_link'
                raw.x = 2.0
                self.raw_pub.publish(raw)
            if self.reverse_points is not None:
                cloud = self.cloud(stamp)
                cloud.width = len(self.reverse_points)
                cloud.row_step = cloud.width * cloud.point_step
                cloud.data = b''.join(struct.pack('<fff', *point)
                                      for point in self.reverse_points)
                self.reverse_pub.publish(cloud)
            rclpy.spin_once(self.node, timeout_sec=0.005)
            for _ in range(8):
                rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(0.015)

    def assert_moving(self):
        """确认近期真实隔离输出已经恢复前进."""
        self.assertGreater(len(self.commands), 0)
        self.assertGreater(self.commands[-1], 0.0)

    def assert_stopped(self):
        """确认近期真实隔离输出保持零速."""
        self.assertGreater(len(self.commands), 2)
        self.assertTrue(all(value == 0.0 for value in self.commands[-3:]))

    def test_dropouts_stale_frames_and_recovery(self):
        """短缺帧保持连续，长断流、过期及重复帧停车，恢复后无需重启."""
        self.pump(0.8)
        self.assert_moving()
        self.commands.clear()
        self.send_cloud = False
        self.pump(0.12)
        self.assertTrue(self.commands)
        self.assertTrue(all(value > 0.0 for value in self.commands))
        self.pump(0.8)
        self.assert_stopped()
        self.send_cloud = True
        self.fixed_stamp = self.node.get_clock().now().to_msg()
        self.fixed_stamp.sec -= 5
        self.pump(0.25)
        self.assert_stopped()
        self.fixed_stamp = None
        self.pump(0.3)
        self.assert_moving()
        # 相同时间戳反复到达不能算作新观测，也不能刷新超时。
        self.fixed_stamp = self.node.get_clock().now().to_msg()
        self.pump(0.95)
        self.assert_stopped()
        self.fixed_stamp = None
        self.pump(0.3)
        self.assert_moving()
        for field in ('send_odom', 'send_nominal'):
            setattr(self, field, False)
            self.pump(0.4)
            self.assert_stopped()
            setattr(self, field, True)
            self.pump(0.35)
            self.assert_moving()

    def test_compensates_obstacles_to_current_pose(self):
        """历史障碍进入当前足迹必须停车，启用机身过滤也不能把它删除."""
        self.pump(0.5)
        self.fixed_stamp = self.node.get_clock().now().to_msg()
        self.point = (0.9, 0.0, 0.2)
        self.pump(0.08)
        self.robot_x = 0.6
        self.states.clear()
        self.pump(0.3)
        self.assert_stopped()
        self.assertIn('BLOCKED', self.states)

    def test_follow_resumes_after_uwb_dropout(self):
        """UWB 中断和旧帧积压会停车，新鲜串口消息恢复后持续跟随自动恢复."""
        self.send_raw = True
        self.pump(0.6)
        self.assertTrue(self.follow_commands)
        self.assertGreater(self.follow_commands[-1], 0.0)
        self.send_raw = False
        self.pump(0.7)
        self.assertEqual(self.follow_commands[-1], 0.0)
        self.send_raw = True
        self.raw_age_sec = 5
        self.pump(0.3)
        self.assertEqual(self.follow_commands[-1], 0.0)
        self.raw_age_sec = 0
        self.pump(0.3)
        self.assertGreater(self.follow_commands[-1], 0.0)

    def test_reverse_retries_without_resetting_distance_budget(self):
        """后方清空后允许重试；图像断流先停车，再恢复且始终保留原距离预算."""
        self.reverse_points = [(0.5, 0.0, 0.2), (-0.65, 0.0, 0.2)]
        self.pump(0.7)
        self.assertTrue(self.reverse_commands)
        self.assertTrue(all(value == 0.0 for value in self.reverse_commands[-3:]))
        self.reverse_points = [(0.5, 0.0, 0.2)]
        self.pump(0.25)
        self.assertLess(self.reverse_commands[-1], 0.0)
        self.reverse_points = None
        self.pump(0.4)
        self.assertEqual(self.reverse_commands[-1], 0.0)
        self.reverse_points = [(0.5, 0.0, 0.2)]
        self.pump(0.3)
        self.assertLess(self.reverse_commands[-1], 0.0)
        self.pump(1.5)
        self.assertTrue(all(value == 0.0 for value in self.reverse_commands[-3:]))
        # 保持触发障碍，反复中断输入也不能重新获得一笔倒退距离。
        self.reverse_points = None
        self.pump(0.8)
        self.reverse_points = [(0.5, 0.0, 0.2)]
        self.reverse_commands.clear()
        self.pump(0.4)
        self.assertTrue(self.reverse_commands)
        self.assertTrue(all(value == 0.0 for value in self.reverse_commands))

    def test_map_retries_after_late_odometry(self):
        """观测先到、里程计后到时自动处理原帧，且保持原始时间戳."""
        self.pump(0.4)
        stamp = self.node.get_clock().now().to_msg()
        # 先发当前里程计建立缓存，再让新观测领先超过允许外推范围。
        self.map_odom_pub.publish(self.odom(stamp))
        self.pump(0.12)
        cloud_stamp = self.node.get_clock().now().to_msg()
        self.map_input_pub.publish(self.cloud(cloud_stamp))
        self.pump(0.06)
        self.assertNotIn(cloud_stamp, self.map_stamps)
        self.map_odom_pub.publish(self.odom(self.node.get_clock().now().to_msg()))
        self.pump(0.12)
        self.assertIn(cloud_stamp, self.map_stamps)


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    """验证节点经历输入故障后仍能正常关闭."""

    def test_exit_codes(self, proc_info, planner, rolling_map, adapter, follow, reverse):
        """所有被测节点必须正常退出."""
        launch_testing.asserts.assertExitCodes(proc_info, process=planner)
        launch_testing.asserts.assertExitCodes(proc_info, process=rolling_map)
        launch_testing.asserts.assertExitCodes(proc_info, process=adapter)
        launch_testing.asserts.assertExitCodes(proc_info, process=follow)
        launch_testing.asserts.assertExitCodes(proc_info, process=reverse)
