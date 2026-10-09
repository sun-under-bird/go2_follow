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

"""验证多标签发布、旧话题隔离及断流诊断."""
import sys
import time
import unittest
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from diagnostic_msgs.msg import DiagnosticArray
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from uwb_aoa_pkg.msg import LibAoaRobotMsg, UwbTarget


# 测试专用时钟不参与部署；采集时钟连续，接收和停止超时仍按真实时间判断。
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))
from ros_test_clock import MonotonicTestClock  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    """启动隔离话题的适配器，避免连接任何真实运动链路."""
    adapter = launch_ros.actions.Node(
        package="go2_uwb_local_follow", executable="uwb_target_adapter_node",
        remappings=[("/clock", "/multi_test/clock")],
        parameters=[str(Path(get_package_share_directory("go2_uwb_local_follow")) /
                        "config" / "uwb_follow_only.yaml"), {
            "use_sim_time": True,
            "raw_topic": "/multi_test/raw", "target_topic": "/multi_test/person",
            "targets_topic": "/multi_test/targets",
            "diagnostics_topic": "/multi_test/diagnostics",
            "target_timeout_sec": 0.20, "diagnostic_frequency": 20.0,
            "sensor_offset_x": 0.1, "sensor_yaw": 1.5707963267948966,
        }],
    )
    return launch.LaunchDescription([
        adapter, launch_testing.actions.ReadyToTest(),
    ]), {"adapter": adapter}


class TestMultiUwb(unittest.TestCase):
    """使用原始消息验证适配器的真实 ROS 回调."""

    @classmethod
    def setUpClass(cls):
        """建立测试上下文."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """关闭测试上下文."""
        rclpy.shutdown()

    def setUp(self):
        """建立测试输入并保存多目标、人员目标和诊断."""
        self.node = rclpy.create_node("multi_uwb_test")
        self.clock = MonotonicTestClock(self.node, "/multi_test/clock")
        self.raw = self.node.create_publisher(LibAoaRobotMsg, "/multi_test/raw", 64)
        self.targets, self.person, self.diagnostics = [], [], []
        self.node.create_subscription(
            UwbTarget, "/multi_test/targets", self.targets.append, 64)
        self.node.create_subscription(
            PointStamped, "/multi_test/person", self.person.append, 10)
        self.node.create_subscription(
            DiagnosticArray, "/multi_test/diagnostics", self.diagnostics.append, 10)

    def tearDown(self):
        """销毁测试节点."""
        self.node.destroy_node()

    def pump(self, seconds):
        """处理订阅回调而不刷新输入，允许真实断流过期."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.clock.sample()
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def send(self, tag, x, y, stamp=None):
        """发送指定 ID 的可信厂家结果，可显式共享时间戳."""
        raw = LibAoaRobotMsg()
        raw.header.stamp = stamp or self.clock.sample()
        raw.fob_id, raw.x, raw.y, raw.pos_confidence = tag, x, y, 100
        self.raw.publish(raw)

    def test_ids_legacy_topic_and_expiration(self):
        """ID2 不得刷新人员话题或延长 ID1 有效期，任意第三 ID 仍可分发."""
        deadline = time.monotonic() + 5.0
        while self.raw.get_subscription_count() < 1 and time.monotonic() < deadline:
            self.pump(0.05)
        self.assertEqual(self.raw.get_subscription_count(), 1)
        self.pump(0.3)
        stamp = self.clock.sample()
        self.send(1, 1.0, 2.0, stamp)
        self.send(2, 3.0, 4.0, stamp)
        self.send(3, 5.0, 6.0, stamp)
        self.pump(0.10)
        targets = {target.target_id: target for target in self.targets}
        self.assertEqual(set(targets), {1, 2, 3})
        self.assertAlmostEqual(targets[1].position.x, -1.9)
        self.assertAlmostEqual(targets[2].position.x, -3.9)
        self.assertAlmostEqual(targets[2].position.y, 3.0)
        self.assertEqual(len(self.person), 1)
        self.assertAlmostEqual(self.person[0].point.x, -1.9)
        for _ in range(5):
            self.send(2, 7.0, 8.0)
            self.pump(0.06)
        self.assertEqual(len(self.person), 1)
        statuses = {s.name: s for s in self.diagnostics[-1].status}
        self.assertIn("STALE", next(v.message for k, v in statuses.items() if k.endswith("ID 1")))
        self.assertEqual(
            next(v.message for k, v in statuses.items() if k.endswith("ID 2")), "UWB_TARGET_VALID")


@launch_testing.post_shutdown_test()
class TestAdapterExit(unittest.TestCase):
    """确认启动框架已回收测试进程."""

    def test_exit(self, proc_info, adapter):
        """适配器应正常退出."""
        launch_testing.asserts.assertExitCodes(proc_info, process=adapter)
