"""UWB target bridge that publishes local follow goals for EXACT-MPPI."""

from __future__ import annotations

import traceback

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from uwb_aoa_pkg.msg import LibAoaRobotMsg

from go2_exact_mppi_follow.goal_logic import (
    FollowGoalConfig,
    UwbParseConfig,
    compute_follow_goal,
    parse_uwb_target,
)
from go2_exact_mppi_follow.ros_geometry import transform_point_xy, yaw_to_quaternion


class Go2UwbGoalBridge(Node):
    """Convert ONE1000/UWB target messages into MPPI local goal poses."""

    def __init__(self) -> None:
        """Initialize parameters, TF, subscriptions, and publishers."""

        super().__init__("go2_uwb_goal_bridge")
        self._declare_parameters()
        self._load_parameters()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.uwb_sub = self.create_subscription(
            LibAoaRobotMsg,
            self.one1000_topic,
            10,
            self._uwb_callback,
        )
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_local_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.get_logger().info(
            f"go2_uwb_goal_bridge started: {self.one1000_topic} -> {self.goal_local_topic}"
        )

    def _declare_parameters(self) -> None:
        """Declare all ROS parameters used by the bridge."""

        self.declare_parameter("one1000_topic", "/libAoa_robot_publisher")
        self.declare_parameter("goal_local_topic", "/exact_mppi/goal_local")
        self.declare_parameter("status_topic", "/exact_mppi/goal_status")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("one1000_frame", "base_link")
        self.declare_parameter("use_tf", True)
        self.declare_parameter("require_tf", True)
        self.declare_parameter("use_latest_tf", True)
        self.declare_parameter("transform_timeout_sec", 0.2)
        self.declare_parameter("require_valid_state", True)
        self.declare_parameter("prefer_xy", True)
        self.declare_parameter("angle_in_degrees", False)
        self.declare_parameter("invert_y", False)
        self.declare_parameter("angle_offset_rad", 0.0)
        self.declare_parameter("anchor_x_offset", 0.0)
        self.declare_parameter("anchor_y_offset", 0.0)
        self.declare_parameter("min_target_distance", 0.15)
        self.declare_parameter("max_target_distance", 8.0)
        self.declare_parameter("follow_distance", 0.9)
        self.declare_parameter("forward_goal_limit", 2.0)
        self.declare_parameter("reverse_goal_limit", 0.4)
        self.declare_parameter("goal_deadband", 0.03)

    def _load_parameters(self) -> None:
        """Load ROS parameters into typed runtime configuration objects."""

        self.one1000_topic = self.get_parameter("one1000_topic").value
        self.goal_local_topic = self.get_parameter("goal_local_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.one1000_frame = self.get_parameter("one1000_frame").value
        self.use_tf = bool(self.get_parameter("use_tf").value)
        self.require_tf = bool(self.get_parameter("require_tf").value)
        self.use_latest_tf = bool(self.get_parameter("use_latest_tf").value)
        self.transform_timeout_sec = float(self.get_parameter("transform_timeout_sec").value)
        self.require_valid_state = bool(self.get_parameter("require_valid_state").value)

        self.uwb_config = UwbParseConfig(
            prefer_xy=bool(self.get_parameter("prefer_xy").value),
            angle_in_degrees=bool(self.get_parameter("angle_in_degrees").value),
            invert_y=bool(self.get_parameter("invert_y").value),
            angle_offset_rad=float(self.get_parameter("angle_offset_rad").value),
            anchor_x_offset=float(self.get_parameter("anchor_x_offset").value),
            anchor_y_offset=float(self.get_parameter("anchor_y_offset").value),
            min_target_distance=float(self.get_parameter("min_target_distance").value),
            max_target_distance=float(self.get_parameter("max_target_distance").value),
        )
        self.goal_config = FollowGoalConfig(
            follow_distance=float(self.get_parameter("follow_distance").value),
            forward_goal_limit=float(self.get_parameter("forward_goal_limit").value),
            reverse_goal_limit=float(self.get_parameter("reverse_goal_limit").value),
            goal_deadband=float(self.get_parameter("goal_deadband").value),
        )

    def _uwb_callback(self, msg: LibAoaRobotMsg) -> None:
        """Parse a UWB target message and publish the corresponding local goal."""

        try:
            if self.require_valid_state and msg.state < 0:
                self._publish_status("reject: invalid UWB state")
                return

            parsed = parse_uwb_target(msg.x, msg.y, msg.r, msg.a, self.uwb_config)
            if parsed is None:
                self._publish_status("reject: invalid UWB target")
                return

            source_frame = msg.header.frame_id or self.one1000_frame
            target_xy = self._target_to_base(parsed, source_frame, msg.header.stamp)
            if target_xy is None:
                self._publish_status("reject: UWB target TF failed")
                return

            goal = compute_follow_goal(target_xy, self.goal_config)
            if goal is None:
                self._publish_status("reject: follow goal unavailable")
                return

            self._publish_goal(goal)
            self._publish_status("ok: goal published")
        except Exception as exc:
            self.get_logger().error(f"UWB goal bridge exception: {exc}\n{traceback.format_exc()}")
            self._publish_status("stop: UWB bridge exception")

    def _target_to_base(self, target_xy, source_frame: str, stamp) -> object:
        """Transform target coordinates into base_frame when needed."""

        if not self.use_tf or source_frame == self.base_frame:
            return target_xy

        try:
            tf_time = Time() if self.use_latest_tf else Time.from_msg(stamp)
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                tf_time,
                timeout=Duration(seconds=self.transform_timeout_sec),
            )
            return transform_point_xy(target_xy, transform)
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f"UWB target TF failed from {source_frame} to {self.base_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            if self.require_tf:
                return None
            return target_xy

    def _publish_goal(self, goal) -> None:
        """Publish a PoseStamped goal in the robot base frame."""

        goal_x, goal_y, goal_yaw = goal
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(goal_x)
        pose.pose.position.y = float(goal_y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(float(goal_yaw))
        self.goal_pub.publish(pose)

    def _publish_status(self, text: str) -> None:
        """Publish compact bridge status messages."""

        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None) -> None:
    """Run the UWB goal bridge node."""

    rclpy.init(args=args)
    node = Go2UwbGoalBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
