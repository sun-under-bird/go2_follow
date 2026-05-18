import rclpy
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool


class FollowPathActionClient(Node):
    def __init__(self):
        super().__init__("follow_path_action_client")

        self.declare_parameter("path_topic", "/follow_path")
        self.declare_parameter("path_valid_topic", "/follow/path_valid")
        self.declare_parameter("action_name", "follow_path")
        self.declare_parameter("controller_id", "FollowPath")
        self.declare_parameter("goal_checker_id", "")
        self.declare_parameter("resend_interval_sec", 0.5)
        self.declare_parameter("minimum_path_poses", 2)

        self.action_client = ActionClient(
            self,
            FollowPath,
            str(self.get_parameter("action_name").value),
        )
        self.latest_path = None
        self.path_valid = False
        self.last_send_time = None
        self.active_goal_handle = None

        self.create_subscription(Path, str(self.get_parameter("path_topic").value), self._path_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter("path_valid_topic").value), self._valid_cb, 10)
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info("follow_path_action_client started")

    def _path_cb(self, msg: Path):
        self.latest_path = msg

    def _valid_cb(self, msg: Bool):
        self.path_valid = bool(msg.data)
        if not self.path_valid:
            self._cancel_active_goal()

    def _cancel_active_goal(self):
        if self.active_goal_handle is not None:
            try:
                self.active_goal_handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().debug(f"FollowPath cancel failed: {exc}")
            self.active_goal_handle = None

    def _ready_to_send(self) -> bool:
        if self.latest_path is None or not self.path_valid:
            return False
        if len(self.latest_path.poses) < int(self.get_parameter("minimum_path_poses").value):
            self._cancel_active_goal()
            return False
        if not self.action_client.server_is_ready():
            self.get_logger().warn("FollowPath action server is not ready", throttle_duration_sec=2.0)
            return False
        if self.last_send_time is None:
            return True
        age = (self.get_clock().now() - self.last_send_time).nanoseconds * 1e-9
        return age >= float(self.get_parameter("resend_interval_sec").value)

    def _tick(self):
        if not self._ready_to_send():
            return

        goal = FollowPath.Goal()
        goal.path = self.latest_path
        goal.controller_id = str(self.get_parameter("controller_id").value)
        goal.goal_checker_id = str(self.get_parameter("goal_checker_id").value)

        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)
        self.last_send_time = self.get_clock().now()

    def _goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"FollowPath goal send failed: {exc}")
            return

        if not goal_handle.accepted:
            self.get_logger().warn("FollowPath goal was rejected", throttle_duration_sec=2.0)
            return

        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        try:
            result = future.result()
            self.get_logger().debug(f"FollowPath result status: {result.status}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().debug(f"FollowPath result failed: {exc}")
        self.active_goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = FollowPathActionClient()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
