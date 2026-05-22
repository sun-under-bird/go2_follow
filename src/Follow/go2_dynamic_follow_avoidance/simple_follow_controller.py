import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .geometry import clamp


class SimpleFollowController(Node):
    def __init__(self):
        super().__init__("simple_follow_controller")

        self.declare_parameter("target_topic", "/one1000/target")
        self.declare_parameter("target_valid_topic", "/follow/target_valid")
        self.declare_parameter("cmd_vel_out", "/cmd_vel_nav")
        self.declare_parameter("status_topic", "/follow/simple_status")
        self.declare_parameter("target_timeout_sec", 0.4)
        self.declare_parameter("target_valid_timeout_sec", 0.5)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("follow_distance", 2.0)
        self.declare_parameter("distance_deadband", 0.15)
        self.declare_parameter("lateral_deadband", 0.08)
        self.declare_parameter("yaw_deadband", 0.08)
        self.declare_parameter("kp_vx", 0.35)
        self.declare_parameter("kp_vy", 0.0)
        self.declare_parameter("kp_yaw", 1.0)
        self.declare_parameter("max_vx", 0.5)
        self.declare_parameter("max_vy", 0.0)
        self.declare_parameter("max_vyaw", 0.8)
        self.declare_parameter("allow_reverse", False)

        self.latest_target: Optional[PoseStamped] = None
        self.latest_target_time = None
        self.target_valid = False
        self.target_valid_time = None
        self.last_status = ""

        self.create_subscription(PoseStamped, str(self.get_parameter("target_topic").value), self._target_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter("target_valid_topic").value), self._valid_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_out").value), 10)
        self.status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info("simple_follow_controller started")

    def _target_cb(self, msg: PoseStamped):
        self.latest_target = msg
        self.latest_target_time = self.get_clock().now()

    def _valid_cb(self, msg: Bool):
        self.target_valid = bool(msg.data)
        self.target_valid_time = self.get_clock().now()

    def _age_ok(self, stamp, timeout_sec: float) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age <= timeout_sec

    def _publish_status(self, status: str):
        if status == self.last_status:
            return
        self.last_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(status)

    def _zero(self) -> Twist:
        return Twist()

    def _tick(self):
        if not self.target_valid or not self._age_ok(
            self.target_valid_time,
            float(self.get_parameter("target_valid_timeout_sec").value),
        ):
            self.cmd_pub.publish(self._zero())
            self._publish_status("stop: UWB target invalid")
            return

        if self.latest_target is None or not self._age_ok(
            self.latest_target_time,
            float(self.get_parameter("target_timeout_sec").value),
        ):
            self.cmd_pub.publish(self._zero())
            self._publish_status("stop: UWB target stale")
            return

        x = float(self.latest_target.pose.position.x)
        y = float(self.latest_target.pose.position.y)
        distance = math.hypot(x, y)
        bearing = math.atan2(y, x) if distance > 1e-6 else 0.0
        follow_distance = float(self.get_parameter("follow_distance").value)
        distance_error = distance - follow_distance

        cmd = Twist()
        if abs(distance_error) > float(self.get_parameter("distance_deadband").value):
            vx = float(self.get_parameter("kp_vx").value) * distance_error
            if not bool(self.get_parameter("allow_reverse").value):
                vx = max(0.0, vx)
            cmd.linear.x = clamp(
                vx,
                -float(self.get_parameter("max_vx").value),
                float(self.get_parameter("max_vx").value),
            )

        if abs(y) > float(self.get_parameter("lateral_deadband").value):
            cmd.linear.y = clamp(
                float(self.get_parameter("kp_vy").value) * y,
                -float(self.get_parameter("max_vy").value),
                float(self.get_parameter("max_vy").value),
            )

        if abs(bearing) > float(self.get_parameter("yaw_deadband").value):
            cmd.angular.z = clamp(
                float(self.get_parameter("kp_yaw").value) * bearing,
                -float(self.get_parameter("max_vyaw").value),
                float(self.get_parameter("max_vyaw").value),
            )

        self.cmd_pub.publish(cmd)
        self._publish_status(f"ok: distance={distance:.2f}m bearing={bearing:.2f}rad")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleFollowController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
