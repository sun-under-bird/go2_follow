import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String

from .geometry import clamp


class SimpleFollowController(Node):
    def __init__(self):
        """初始化直接消费原始目标并输出 /cmd_vel 的简化控制器。"""
        super().__init__("simple_follow_controller")

        self.declare_parameter("target_topic", "/one1000/target")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/follow/simple_status")
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
        self.last_status = ""

        self.create_subscription(PoseStamped, str(self.get_parameter("target_topic").value), self._target_cb, 10)
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info("simple_follow_controller started")

    def _target_cb(self, msg: PoseStamped):
        """直接保存最近一次原始目标，不做有效标志或超时门控。"""
        self.latest_target = msg

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
        """根据最近一次原始目标直接计算并发布速度。"""
        if self.latest_target is None:
            self.cmd_pub.publish(self._zero())
            self._publish_status("stop: waiting for UWB target")
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
