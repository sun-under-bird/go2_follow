"""Verify target-only service behavior without any chassis velocity output."""
import time
import unittest

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy

from go2_uwb_behavior.srv import GenerateNavigationGoal, SetBehavior


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        ),
        launch_ros.actions.Node(
            package="go2_uwb_behavior", executable="uwb_behavior_controller_node",
            name="uwb_navigation_target_node",
            parameters=[{"target_only": True, "readiness_timeout_sec": 1.,
                         "uwb_median_window": 3, "minimum_owner_samples": 3}],
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestNavigationTarget(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("navigation_target_test")
        self.target = self.node.create_publisher(PointStamped, "/uwb/target_point", 10)
        self.odom = self.node.create_publisher(Odometry, "/leg_odom2", 10)
        self.cloud = self.node.create_publisher(PointCloud2, "/local_rolling_obstacle", 10)
        self.client = self.node.create_client(GenerateNavigationGoal, "/go2/generate_navigation_goal")
        self.mode = self.node.create_client(SetBehavior, "/go2/set_behavior")
        assert self.client.wait_for_service(timeout_sec=5.)

    def tearDown(self):
        self.node.destroy_node()

    def inputs(self):
        stamp = self.node.get_clock().now().to_msg()
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
        deadline = time.monotonic() + 5.
        while not future.done() and time.monotonic() < deadline:
            if inputs:
                self.inputs()
            rclpy.spin_once(self.node, timeout_sec=.02)
        assert future.done()
        return future.result()

    def request(self, request_id="test-1", minimum=.5, maximum=2.):
        return GenerateNavigationGoal.Request(
            request_id=request_id, random_seed=17, min_radius=minimum, max_radius=maximum,
        )

    def test_generates_map_goal_and_never_publishes_velocity(self):
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
        response = self.wait(self.client.call_async(self.request("", -1., 2.)))
        assert not response.success and response.code == response.INVALID_REQUEST
        first = self.client.call_async(self.request("first"))
        second = self.client.call_async(self.request("second"))
        busy = self.wait(second, inputs=False)
        assert busy.request_id == "second" and busy.code == busy.BUSY
        assert self.wait(first).success

    def test_missing_inputs_times_out_without_a_goal(self):
        time.sleep(1.)
        response = self.wait(self.client.call_async(self.request()), inputs=False)
        assert not response.success and response.code == response.INPUT_TIMEOUT
        assert self.node.count_publishers("/cmd_vel") == 0
