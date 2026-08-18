import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from .geometry import clamp, transform_from_ros, transform_point
from .safety_helpers import limit_delta


class SafetyMux(Node):
    def __init__(self):
        """初始化候选速度输入、传感器看门狗和唯一的最终速度发布口。"""
        super().__init__("safety_mux")

        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("cmd_vel_in", "/cmd_vel_nav")
        self.declare_parameter("cmd_vel_out", "/cmd_vel")
        self.declare_parameter("pointcloud_topic", "/local_grid_obstacle")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("path_topic", "/follow_path")
        self.declare_parameter("target_valid_topic", "/follow/target_valid")
        self.declare_parameter("path_valid_topic", "/follow/path_valid")
        self.declare_parameter("bypass_safety", False)
        self.declare_parameter("require_odom_watchdog", True)
        self.declare_parameter("require_path_watchdog", False)
        self.declare_parameter("require_pointcloud_watchdog", True)
        self.declare_parameter("cmd_timeout_sec", 0.3)
        self.declare_parameter("odom_timeout_sec", 0.5)
        self.declare_parameter("pointcloud_timeout_sec", 0.5)
        self.declare_parameter("target_valid_timeout_sec", 0.5)
        self.declare_parameter("path_valid_timeout_sec", 0.5)
        self.declare_parameter("path_timeout_sec", 0.5)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("max_vx", 0.2)
        self.declare_parameter("max_vy", 0.3)
        self.declare_parameter("max_vyaw", 0.8)
        self.declare_parameter("max_reverse_vx", 0.0)
        self.declare_parameter("max_accel_vx", 0.6)
        self.declare_parameter("max_accel_vy", 0.8)
        self.declare_parameter("max_accel_vyaw", 1.5)
        self.declare_parameter("emergency_x_max", 0.45)
        self.declare_parameter("obstacle_clear_x_max", 0.60)
        self.declare_parameter("slow_x_max", 1.0)
        self.declare_parameter("front_y_abs", 0.45)
        self.declare_parameter("obstacle_z_min", 0.05)
        self.declare_parameter("obstacle_z_max", 1.2)
        self.declare_parameter("max_points_per_cloud", 60000)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_cmd: Optional[Twist] = None
        self.latest_cmd_time = None
        self.latest_cloud_time = None
        self.latest_odom_time = None
        self.latest_path_time = None
        self.latest_path_pose_count = 0
        self.target_valid_time = None
        self.path_valid_time = None
        self.nearest_front_obstacle: Optional[float] = None
        self.target_valid = False
        self.path_valid = False
        self.obstacle_stop_latched = False
        self.last_status = ""
        self.last_output = Twist()
        self.last_tick_time = self.get_clock().now()

        self.create_subscription(Twist, str(self.get_parameter("cmd_vel_in").value), self._cmd_cb, 10)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("pointcloud_topic").value),
            self._cloud_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, 10)
        self.create_subscription(Path, str(self.get_parameter("path_topic").value), self._path_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter("target_valid_topic").value), self._target_valid_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter("path_valid_topic").value), self._path_valid_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_vel_out").value), 10)
        self.status_pub = self.create_publisher(String, "/follow/safety_status", 10)

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info("safety_mux started")

    def _cmd_cb(self, msg: Twist):
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()

    def _odom_cb(self, _msg: Odometry):
        self.latest_odom_time = self.get_clock().now()

    def _path_cb(self, msg: Path):
        self.latest_path_time = self.get_clock().now()
        self.latest_path_pose_count = len(msg.poses)

    def _target_valid_cb(self, msg: Bool):
        self.target_valid = bool(msg.data)
        self.target_valid_time = self.get_clock().now()

    def _path_valid_cb(self, msg: Bool):
        self.path_valid = bool(msg.data)
        self.path_valid_time = self.get_clock().now()

    def _cloud_cb(self, msg: PointCloud2):
        transform = None
        if msg.header.frame_id and msg.header.frame_id != self.base_frame:
            try:
                transform = transform_from_ros(
                    self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"Safety point cloud TF failed: {exc}", throttle_duration_sec=2.0)
                return

        emergency_x_max = float(self.get_parameter("emergency_x_max").value)
        clear_x_max = float(self.get_parameter("obstacle_clear_x_max").value)
        slow_x_max = float(self.get_parameter("slow_x_max").value)
        front_y_abs = float(self.get_parameter("front_y_abs").value)
        z_min = float(self.get_parameter("obstacle_z_min").value)
        z_max = float(self.get_parameter("obstacle_z_max").value)
        max_points = int(self.get_parameter("max_points_per_cloud").value)

        nearest = None
        seen = 0
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if seen >= max_points:
                break
            seen += 1
            px, py, pz = float(point[0]), float(point[1]), float(point[2])
            if transform is not None:
                px, py, pz = transform_point((px, py, pz), transform)
            if not (0.0 <= px <= slow_x_max):
                continue
            if abs(py) > front_y_abs:
                continue
            if not (z_min <= pz <= z_max):
                continue
            nearest = px if nearest is None else min(nearest, px)
            if nearest <= emergency_x_max:
                break

        if nearest is not None and nearest <= emergency_x_max:
            self.obstacle_stop_latched = True
        elif self.obstacle_stop_latched and (nearest is None or nearest >= clear_x_max):
            self.obstacle_stop_latched = False

        self.nearest_front_obstacle = nearest
        self.latest_cloud_time = self.get_clock().now()

    def _age_ok(self, stamp, timeout_sec: float) -> bool:
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        return age <= timeout_sec

    def _zero(self) -> Twist:
        return Twist()

    def _publish_status(self, status: str):
        if status == self.last_status:
            return
        self.last_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(status)

    def _clip_cmd(self, cmd: Twist) -> Twist:
        out = Twist()
        out.linear.x = clamp(
            cmd.linear.x,
            -float(self.get_parameter("max_reverse_vx").value),
            float(self.get_parameter("max_vx").value),
        )
        out.linear.y = clamp(
            cmd.linear.y,
            -float(self.get_parameter("max_vy").value),
            float(self.get_parameter("max_vy").value),
        )
        out.angular.z = clamp(
            cmd.angular.z,
            -float(self.get_parameter("max_vyaw").value),
            float(self.get_parameter("max_vyaw").value),
        )
        return out

    def _ramp_cmd(self, target: Twist, dt: float) -> Twist:
        out = Twist()
        out.linear.x = limit_delta(
            self.last_output.linear.x,
            target.linear.x,
            float(self.get_parameter("max_accel_vx").value) * dt,
        )
        out.linear.y = limit_delta(
            self.last_output.linear.y,
            target.linear.y,
            float(self.get_parameter("max_accel_vy").value) * dt,
        )
        out.angular.z = limit_delta(
            self.last_output.angular.z,
            target.angular.z,
            float(self.get_parameter("max_accel_vyaw").value) * dt,
        )
        return out

    def _sanitize_zero(self, output: Twist) -> Twist:
        if math.isclose(output.linear.x, 0.0, abs_tol=1e-6):
            output.linear.x = 0.0
        if math.isclose(output.linear.y, 0.0, abs_tol=1e-6):
            output.linear.y = 0.0
        if math.isclose(output.angular.z, 0.0, abs_tol=1e-6):
            output.angular.z = 0.0
        return output

    def _tick(self):
        now = self.get_clock().now()
        dt = max(1e-3, (now - self.last_tick_time).nanoseconds * 1e-9)
        self.last_tick_time = now

        status = "ok"
        output = self._zero()

        if bool(self.get_parameter("bypass_safety").value):
            if not self.target_valid or not self._age_ok(
                self.target_valid_time, float(self.get_parameter("target_valid_timeout_sec").value)
            ):
                status = "bypass: target stop or invalid"
                output = self._zero()
            elif not self._age_ok(self.latest_cmd_time, float(self.get_parameter("cmd_timeout_sec").value)) or self.latest_cmd is None:
                status = "bypass: /cmd_vel_nav stale"
                output = self._zero()
            else:
                status = "bypass: safety disabled"
                output = self._clip_cmd(self.latest_cmd)
        elif not self.target_valid or not self._age_ok(
            self.target_valid_time, float(self.get_parameter("target_valid_timeout_sec").value)
        ):
            status = "stop: UWB target invalid or stale"
        elif bool(self.get_parameter("require_path_watchdog").value) and (
            not self.path_valid
            or not self._age_ok(
                self.path_valid_time,
                float(self.get_parameter("path_valid_timeout_sec").value),
            )
        ):
            status = "stop: no valid local path"
        elif bool(self.get_parameter("require_path_watchdog").value) and not self._age_ok(
            self.latest_path_time,
            float(self.get_parameter("path_timeout_sec").value),
        ):
            status = "stop: follow path stale"
        elif bool(self.get_parameter("require_path_watchdog").value) and self.latest_path_pose_count < 1:
            status = "stop: follow path empty"
        elif bool(self.get_parameter("require_odom_watchdog").value) and not self._age_ok(
            self.latest_odom_time,
            float(self.get_parameter("odom_timeout_sec").value),
        ):
            status = "stop: odom stale"
        elif bool(self.get_parameter("require_pointcloud_watchdog").value) and not self._age_ok(
            self.latest_cloud_time,
            float(self.get_parameter("pointcloud_timeout_sec").value),
        ):
            status = "stop: RTAB-Map obstacle cloud stale"
        elif not self._age_ok(self.latest_cmd_time, float(self.get_parameter("cmd_timeout_sec").value)) or self.latest_cmd is None:
            status = "stop: /cmd_vel_nav stale"
        elif self.obstacle_stop_latched:
            if self.nearest_front_obstacle is None:
                status = "stop: obstacle latch"
            else:
                status = f"stop: obstacle at {self.nearest_front_obstacle:.2f}m"
        else:
            target = self._clip_cmd(self.latest_cmd)
            emergency_x_max = float(self.get_parameter("emergency_x_max").value)
            slow_x_max = float(self.get_parameter("slow_x_max").value)
            if (
                self.nearest_front_obstacle is not None
                and emergency_x_max < self.nearest_front_obstacle <= slow_x_max
                and target.linear.x > 0.0
            ):
                scale = (self.nearest_front_obstacle - emergency_x_max) / max(0.01, slow_x_max - emergency_x_max)
                target.linear.x *= clamp(scale, 0.0, 1.0)
                status = f"slow: obstacle at {self.nearest_front_obstacle:.2f}m"
            output = self._ramp_cmd(target, dt)

        if status.startswith("stop:"):
            self.obstacle_stop_latched = self.obstacle_stop_latched and self._age_ok(
                self.latest_cloud_time, float(self.get_parameter("pointcloud_timeout_sec").value)
            )
            output = self._zero()

        output = self._sanitize_zero(output)
        self.last_output = output
        self.cmd_pub.publish(output)
        self._publish_status(status)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
