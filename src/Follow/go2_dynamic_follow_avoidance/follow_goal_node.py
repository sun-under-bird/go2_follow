import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from .geometry import transform_from_ros, transform_point, yaw_to_quaternion
from .ros_utils import first_existing_attr, import_message_type, to_float


class FollowGoalNode(Node):
    def __init__(self):
        super().__init__("follow_goal_node")

        self.declare_parameter("one1000_topic", "/libAoa_robot_publisher")
        self.declare_parameter("one1000_msg_type", "uwb_aoa_pkg/msg/LibAoaRobot")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("one1000_frame", "one1000_anchor")
        self.declare_parameter("use_tf", True)
        self.declare_parameter("follow_distance", 1.5)
        self.declare_parameter("goal_tolerance", 0.15)
        self.declare_parameter("confidence_threshold", 50.0)
        self.declare_parameter("target_timeout_sec", 0.3)
        self.declare_parameter("angle_units", "rad")
        self.declare_parameter("x_fields", ["x", "pos_x", "position.x"])
        self.declare_parameter("y_fields", ["y", "pos_y", "position.y"])
        self.declare_parameter("r_fields", ["r", "distance", "range"])
        self.declare_parameter("angle_fields", ["a", "angle", "rad", "azimuth"])
        self.declare_parameter("state_fields", ["state"])
        self.declare_parameter("confidence_fields", ["pos_confidence", "confidence"])
        self.declare_parameter("invert_y", False)
        self.declare_parameter("angle_offset_rad", 0.0)
        self.declare_parameter("anchor_x_offset", 0.0)
        self.declare_parameter("anchor_y_offset", 0.0)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.one1000_frame = self.get_parameter("one1000_frame").value
        self.use_tf = bool(self.get_parameter("use_tf").value)
        self.follow_distance = float(self.get_parameter("follow_distance").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.target_timeout_sec = float(self.get_parameter("target_timeout_sec").value)
        self.angle_units = str(self.get_parameter("angle_units").value).lower()
        self.invert_y = bool(self.get_parameter("invert_y").value)
        self.angle_offset_rad = float(self.get_parameter("angle_offset_rad").value)
        self.anchor_x_offset = float(self.get_parameter("anchor_x_offset").value)
        self.anchor_y_offset = float(self.get_parameter("anchor_y_offset").value)

        msg_type = import_message_type(str(self.get_parameter("one1000_msg_type").value))
        self.sub = self.create_subscription(
            msg_type,
            str(self.get_parameter("one1000_topic").value),
            self._target_cb,
            10,
        )

        self.goal_pub = self.create_publisher(PoseStamped, "/follow_goal", 10)
        self.target_pub = self.create_publisher(PoseStamped, "/one1000/target", 10)
        self.valid_pub = self.create_publisher(Bool, "/follow/target_valid", 10)

        self.tf_buffer: Optional[Buffer] = Buffer() if self.use_tf else None
        self.tf_listener: Optional[TransformListener] = (
            TransformListener(self.tf_buffer, self) if self.use_tf else None
        )

        self.last_target_xy: Optional[Tuple[float, float]] = None
        self.last_target_time = None
        self.last_target_valid = False

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self._publish)

        self.get_logger().info("follow_goal_node started")

    def _fields(self, name: str):
        return list(self.get_parameter(name).value)

    def _target_cb(self, msg):
        parsed = self._parse_one1000(msg)
        if parsed is None:
            self.last_target_valid = False
            return

        x, y, state, confidence = parsed
        valid = state >= 0 and confidence >= self.confidence_threshold
        if valid:
            self.last_target_xy = (x, y)
            self.last_target_time = self.get_clock().now()
        self.last_target_valid = valid

    def _parse_one1000(self, msg) -> Optional[Tuple[float, float, int, float]]:
        x = to_float(first_existing_attr(msg, self._fields("x_fields")))
        y = to_float(first_existing_attr(msg, self._fields("y_fields")))

        if x is None or y is None:
            r = to_float(first_existing_attr(msg, self._fields("r_fields")))
            angle = to_float(first_existing_attr(msg, self._fields("angle_fields")))
            if r is None or angle is None:
                self.get_logger().warn("ONE1000 message has no x/y or r/angle fields", throttle_duration_sec=2.0)
                return None
            if self.angle_units.startswith("deg"):
                angle = math.radians(angle)
            angle += self.angle_offset_rad
            x = r * math.cos(angle)
            y = r * math.sin(angle)
        else:
            x += self.anchor_x_offset
            y += self.anchor_y_offset

        if self.invert_y:
            y = -y

        state = int(to_float(first_existing_attr(msg, self._fields("state_fields")), 0.0) or 0.0)
        confidence = to_float(
            first_existing_attr(msg, self._fields("confidence_fields")),
            100.0,
        )
        if confidence is None:
            confidence = 100.0
        return float(x), float(y), state, float(confidence)

    def _target_is_fresh(self) -> bool:
        if self.last_target_time is None:
            return False
        age = (self.get_clock().now() - self.last_target_time).nanoseconds * 1e-9
        return age <= self.target_timeout_sec

    def _transform_target_to_base(self, x: float, y: float) -> Tuple[float, float]:
        if not self.use_tf or self.one1000_frame == self.base_frame:
            return x, y
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.one1000_frame, Time())
            bx, by, _ = transform_point((x, y, 0.0), transform_from_ros(tf))
            return bx, by
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"Using ONE1000 raw coordinates because TF failed: {exc}",
                throttle_duration_sec=2.0,
            )
            return x, y

    def _make_pose(self, x: float, y: float, frame_id: str) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        yaw = math.atan2(y, x) if abs(x) + abs(y) > 1e-6 else 0.0
        pose.pose.orientation = yaw_to_quaternion(yaw)
        return pose

    def _publish(self):
        valid = self.last_target_valid and self._target_is_fresh() and self.last_target_xy is not None
        valid_msg = Bool()
        valid_msg.data = bool(valid)
        self.valid_pub.publish(valid_msg)

        if not valid:
            return

        target_x, target_y = self._transform_target_to_base(*self.last_target_xy)
        self.target_pub.publish(self._make_pose(target_x, target_y, self.base_frame))

        distance = math.hypot(target_x, target_y)
        if distance <= self.follow_distance + self.goal_tolerance:
            goal_x = 0.0
            goal_y = 0.0
        else:
            travel = max(0.0, distance - self.follow_distance)
            goal_x = travel * target_x / distance
            goal_y = travel * target_y / distance

        self.goal_pub.publish(self._make_pose(goal_x, goal_y, self.base_frame))


def main(args=None):
    rclpy.init(args=args)
    node = FollowGoalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
