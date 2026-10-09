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

"""验证纯目标服务和底盘速度隔离，保留原有断言."""
import sys
import time
import unittest
from pathlib import Path

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy

from go2_uwb_behavior.srv import GenerateNavigationGoal, SetBehavior


# 测试专用时钟不参与部署；采集时钟连续，接收和停止超时仍按真实时间判断。
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))
from ros_test_clock import MonotonicTestClock  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    """提供固定 map 变换并启动纯目标节点."""
    return launch.LaunchDescription([
        # 固定地图变换用于验证服务返回的 map 目标。
        launch_ros.actions.Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        ),
        # 此节点只生成目标，不创建名义或最终速度发布者。
        launch_ros.actions.Node(
            package="go2_uwb_behavior", executable="uwb_behavior_controller_node",
            name="uwb_navigation_target_node",
            remappings=[("/clock", "/navigation_test/clock")],
            parameters=[{"use_sim_time": True, "target_only": True, "readiness_timeout_sec": 1.,
                         "uwb_median_window": 3, "minimum_owner_samples": 3}],
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestNavigationTarget(unittest.TestCase):
    """检查目标生成、请求串行化和输入超时."""

    @classmethod
    def setUpClass(cls):
        """初始化 ROS 上下文."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """结束 ROS 上下文."""
        rclpy.shutdown()

    def setUp(self):
        """建立模拟采集输入和服务客户端."""
        self.node = rclpy.create_node("navigation_target_test")
        self.clock = MonotonicTestClock(self.node, "/navigation_test/clock")
        self.target = self.node.create_publisher(PointStamped, "/uwb/target_point", 10)
        self.odom = self.node.create_publisher(Odometry, "/leg_odom2", 10)
        self.cloud = self.node.create_publisher(PointCloud2, "/local_rolling_obstacle", 10)
        self.client = self.node.create_client(
            GenerateNavigationGoal, "/go2/generate_navigation_goal")
        self.mode = self.node.create_client(SetBehavior, "/go2/set_behavior")
        assert self.client.wait_for_service(timeout_sec=5.)

    def tearDown(self):
        """销毁本用例客户端节点."""
        self.node.destroy_node()

    def inputs(self):
        """持续发送采集时间一致的人员、里程计和空点云."""
        stamp = self.clock.sample()
        target = PointStamped()
        target.header.stamp, target.header.frame_id = stamp, "base_footprint"
        target.point.x = 1.
        self.target.publish(target)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.w = 1.
        self.odom.publish(odom)
        cloud = PointCloud2()
        cloud.header.stamp, cloud.header.frame_id = stamp, "base_footprint"
        cloud.height, cloud.width = 1, 0
        cloud.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1)]
        cloud.point_step, cloud.row_step, cloud.is_dense = 8, 0, True
        self.cloud.publish(cloud)

    def wait(self, future, *, inputs=True):
        """按需持续供数并等待服务回复."""
        deadline = time.monotonic() + 5.
        while not future.done() and time.monotonic() < deadline:
            self.clock.sample()
            if inputs:
                self.inputs()
            rclpy.spin_once(self.node, timeout_sec=.02)
        assert future.done()
        return future.result()

    def request(self, request_id="test-1", minimum=.5, maximum=2.):
        """创建半径和随机源固定的目标请求."""
        return GenerateNavigationGoal.Request(
            request_id=request_id, random_seed=17, min_radius=minimum, max_radius=maximum,
        )

    def test_generates_map_goal_and_never_publishes_velocity(self):
        """目标必须位于 map 圆环内，且不能产生运动发布者."""
        response = self.wait(self.client.call_async(self.request()))
        assert response.success and response.code == response.SUCCESS
        assert response.request_id == "test-1"
        assert response.navigation_goal.header.frame_id == "map"
        assert response.center_pose.header.frame_id == "map"
        p, c = response.navigation_goal.pose.position, response.center_pose.pose.position
        assert .5 <= ((p.x - c.x) ** 2 + (p.y - c.y) ** 2) ** .5 <= 2.
        assert self.node.count_publishers("/cmd_vel") == 0
        assert self.node.count_publishers("/go2_uwb_local_follow/nominal_cmd") == 0

    def test_rejects_invalid_request_and_serializes_requests(self):
        """非法请求被拒绝，已有请求期间的新请求返回忙."""
        response = self.wait(self.client.call_async(self.request("", -1., 2.)))
        assert not response.success and response.code == response.INVALID_REQUEST
        first = self.client.call_async(self.request("first"))
        second = self.client.call_async(self.request("second"))
        busy = self.wait(second, inputs=False)
        assert busy.request_id == "second" and busy.code == busy.BUSY
        assert self.wait(first).success

    def test_missing_inputs_times_out_without_a_goal(self):
        """输入断流后不能复用旧数据生成新目标."""
        time.sleep(1.)
        response = self.wait(self.client.call_async(self.request()), inputs=False)
        assert not response.success and response.code == response.INPUT_TIMEOUT
        assert self.node.count_publishers("/cmd_vel") == 0
