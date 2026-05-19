from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String


class Nav2DynamicFollowClient(Node):
    def __init__(self):
        super().__init__("nav2_dynamic_follow_client")

        self.declare_parameter("goal_update_topic", "/goal_update")
        self.declare_parameter("target_valid_topic", "/follow/target_valid")
        self.declare_parameter("path_valid_topic", "/follow/path_valid")
        self.declare_parameter("status_topic", "/follow/nav2_status")
        self.declare_parameter("navigate_action_name", "navigate_to_pose")
        self.declare_parameter("behavior_tree", "")
        self.declare_parameter("server_wait_timeout_sec", 1.0)
        self.declare_parameter("resend_after_result_sec", 0.3)
        self.declare_parameter("publish_rate_hz", 10.0)

        self.latest_goal: Optional[PoseStamped] = None
        self.target_valid = False
        self.target_valid_time = None
        self.active_goal_handle = None
        self.goal_in_flight = False
        self.last_result_time = None
        self.last_status = ""

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigate_action_name").value),
        )

        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_update_topic").value),
            self._goal_cb,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("target_valid_topic").value),
            self._target_valid_cb,
            10,
        )

        self.path_valid_pub = self.create_publisher(Bool, str(self.get_parameter("path_valid_topic").value), 10)
        self.status_pub = self.create_publisher(String, str(self.get_parameter("status_topic").value), 10)

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info("nav2_dynamic_follow_client started")

    def _goal_cb(self, msg: PoseStamped):
        self.latest_goal = msg

    def _target_valid_cb(self, msg: Bool):
        self.target_valid = bool(msg.data)
        self.target_valid_time = self.get_clock().now()
        if not self.target_valid:
            self._cancel_active_goal("target invalid")

    def _publish_path_valid(self, valid: bool):
        msg = Bool()
        msg.data = valid
        self.path_valid_pub.publish(msg)

    def _publish_status(self, status: str):
        if status == self.last_status:
            return
        self.last_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(status)

    def _ready_to_send(self) -> bool:
        if not self.target_valid:
            self._publish_status("stop: UWB target invalid")
            return False
        if self.latest_goal is None:
            self._publish_status("waiting: no /goal_update")
            return False
        if self.active_goal_handle is not None or self.goal_in_flight:
            return False
        if self.last_result_time is not None:
            age = (self.get_clock().now() - self.last_result_time).nanoseconds * 1e-9
            if age < float(self.get_parameter("resend_after_result_sec").value):
                return False
        if not self.action_client.wait_for_server(
            timeout_sec=float(self.get_parameter("server_wait_timeout_sec").value)
        ):
            self._publish_status("waiting: navigate_to_pose action server")
            return False
        return True

    def _tick(self):
        self._publish_path_valid(self.target_valid and self.active_goal_handle is not None)
        if self._ready_to_send():
            self._send_goal()

    def _send_goal(self):
        goal = NavigateToPose.Goal()
        goal.pose = self.latest_goal
        goal.behavior_tree = str(self.get_parameter("behavior_tree").value)

        self.goal_in_flight = True
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)
        self._publish_status("running: sent NavigateToPose")

    def _goal_response_cb(self, future):
        self.goal_in_flight = False
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._publish_status(f"stop: NavigateToPose send failed: {exc}")
            return

        if not goal_handle.accepted:
            self._publish_status("stop: NavigateToPose rejected")
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)
        self._publish_status("running: NavigateToPose active")

    def _result_cb(self, future):
        status = "done"
        try:
            result = future.result()
            status = f"done: NavigateToPose status {result.status}"
        except Exception as exc:  # noqa: BLE001
            status = f"done: NavigateToPose result failed: {exc}"
        self.active_goal_handle = None
        self.last_result_time = self.get_clock().now()
        self._publish_status(status)

    def _cancel_active_goal(self, reason: str):
        if self.active_goal_handle is not None:
            try:
                self.active_goal_handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(f"NavigateToPose cancel failed: {exc}")
            self.active_goal_handle = None
        self.goal_in_flight = False
        self._publish_path_valid(False)
        self._publish_status(f"stop: {reason}")


def main(args=None):
    rclpy.init(args=args)
    node = Nav2DynamicFollowClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
