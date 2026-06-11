"""ROS2 adapter for the EXACT-MPPI JAX controller using stereo PointCloud2 input.

This node is derived from the public behavior of EXACT-MPPI's ROS2 mppi_local.py:
https://github.com/caseypen/EXACT-mppi/blob/main/mosaic_mppi_ros2/src/exact_mppi_jax/exact_mppi_jax/mppi_local.py
It keeps the same core controller call, but replaces LaserScan input with filtered stereo PointCloud2.
"""

from __future__ import annotations

import os
import struct
import threading
import time
import traceback
from typing import Dict, Optional

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
import yaml

from go2_exact_mppi_follow.cloud_filter import (
    CloudFilterConfig,
    filter_cloud_points,
    rotation_matrix_from_quaternion,
    transform_points,
)
from go2_exact_mppi_follow.ros_geometry import quaternion_to_yaw, yaw_to_quaternion

try:
    import jax
except ImportError:
    jax = None

try:
    from exact_mppi.mppi_jax.controller import MPPIController
except ImportError as import_error:
    MPPIController = None
    MPPI_IMPORT_ERROR = import_error
else:
    MPPI_IMPORT_ERROR = None


class Go2ExactMppiNode(Node):
    """Run EXACT-MPPI with a local UWB goal and stereo point-cloud obstacles."""

    def __init__(self) -> None:
        """Initialize ROS interfaces, config, and the JAX MPPI controller."""

        super().__init__("go2_exact_mppi_node", automatically_declare_parameters_from_overrides=True)
        self.state_lock = threading.Lock()
        self.mppi_lock = threading.Lock()
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.callback_group = ReentrantCallbackGroup()
        self._declare_default_parameters()
        self._load_parameters()
        self._check_runtime()
        self._load_mppi_config()
        self._build_controller()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_goal: Optional[np.ndarray] = None
        self.latest_goal_time = self.get_clock().now()
        self.latest_points: Optional[np.ndarray] = None
        self.latest_cloud_time = self.get_clock().now()
        self.latest_speed = np.zeros((3,), dtype=np.float32)
        self.latest_odom_time = self.get_clock().now()
        self.have_odom = False
        self.cost_keys: Optional[list[str]] = None
        self.tick_count = 0

        self._create_ros_interfaces()
        self.get_logger().info(
            f"go2_exact_mppi_node ready: control={self.control_frequency}Hz "
            f"backend={self.jax_backend}"
        )

    def _declare_default_parameters(self) -> None:
        """Declare defaults for parameters not supplied through YAML."""

        defaults = {
            "mppi_config_file": "go2_orin_omni.yaml",
            "require_jax_gpu": True,
            "base_frame": "base_link",
            "map_frame": "odom",
            "pointcloud_topic": "/stereo/points2",
            "odom_topic": "/odom",
            "goal_local_topic": "/exact_mppi/goal_local",
            "cmd_vel_topic": "/cmd_vel",
            "cmd_vel_raw_topic": "/exact_mppi/cmd_vel_raw",
            "filtered_points_topic": "/exact_mppi/filtered_points",
            "status_topic": "/exact_mppi/status",
            "cost_breakdown_topic": "/exact_mppi/cost_breakdown",
            "plan_topic": "/exact_mppi/plan",
            "control_frequency": 10.0,
            "goal_timeout_sec": 1.0,
            "cloud_timeout_sec": 0.5,
            "odom_timeout_sec": 0.5,
            "transform_timeout_sec": 0.2,
            "use_latest_tf": True,
            "max_raw_points": 60000,
            "emergency_stop_distance": 0.18,
            "publish_plan": True,
            "publish_cost_breakdown": True,
            "timing_log_every_n": 20,
            "cloud_x_min": -0.4,
            "cloud_x_max": 3.0,
            "cloud_y_abs": 1.8,
            "cloud_z_min": 0.05,
            "cloud_z_max": 1.0,
            "voxel_size": 0.08,
            "corridor_width": 0.9,
        }
        for name, default in defaults.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, default)

    def _load_parameters(self) -> None:
        """Load node-level ROS parameters into member variables."""

        self.mppi_config_file = str(self.get_parameter("mppi_config_file").value)
        self.require_jax_gpu = bool(self.get_parameter("require_jax_gpu").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.goal_local_topic = str(self.get_parameter("goal_local_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.cmd_vel_raw_topic = str(self.get_parameter("cmd_vel_raw_topic").value)
        self.filtered_points_topic = str(self.get_parameter("filtered_points_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.cost_breakdown_topic = str(self.get_parameter("cost_breakdown_topic").value)
        self.plan_topic = str(self.get_parameter("plan_topic").value)
        self.control_frequency = float(self.get_parameter("control_frequency").value)
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self.cloud_timeout_sec = float(self.get_parameter("cloud_timeout_sec").value)
        self.odom_timeout_sec = float(self.get_parameter("odom_timeout_sec").value)
        self.transform_timeout_sec = float(self.get_parameter("transform_timeout_sec").value)
        self.use_latest_tf = bool(self.get_parameter("use_latest_tf").value)
        self.max_raw_points = int(self.get_parameter("max_raw_points").value)
        self.emergency_stop_distance = float(self.get_parameter("emergency_stop_distance").value)
        self.publish_plan_enabled = bool(self.get_parameter("publish_plan").value)
        self.publish_cost_breakdown_enabled = bool(self.get_parameter("publish_cost_breakdown").value)
        self.timing_log_every_n = max(1, int(self.get_parameter("timing_log_every_n").value))
        self.cloud_config = CloudFilterConfig(
            x_min=float(self.get_parameter("cloud_x_min").value),
            x_max=float(self.get_parameter("cloud_x_max").value),
            y_abs=float(self.get_parameter("cloud_y_abs").value),
            z_min=float(self.get_parameter("cloud_z_min").value),
            z_max=float(self.get_parameter("cloud_z_max").value),
            voxel_size=float(self.get_parameter("voxel_size").value),
            max_points=300,
            corridor_width=float(self.get_parameter("corridor_width").value),
        )

    def _check_runtime(self) -> None:
        """Verify that exact_mppi and the requested JAX backend are available."""

        if MPPIController is None:
            raise ImportError(
                "Failed to import exact_mppi.mppi_jax.controller.MPPIController. "
                "Install third_party/EXACT-mppi/EXACT_MPPI_core into this Python environment."
            ) from MPPI_IMPORT_ERROR

        if jax is None:
            raise ImportError("JAX is required by EXACT-MPPI but is not installed.")

        self.jax_backend = str(jax.default_backend())
        self.jax_devices = [str(device) for device in jax.devices()]
        if self.require_jax_gpu and self.jax_backend not in ("gpu", "cuda"):
            raise RuntimeError(
                f"JAX backend is {self.jax_backend}, devices={self.jax_devices}. "
                "Fix JetPack/JAX/CUDA before real-time Orin Nano testing."
            )

    def _load_mppi_config(self) -> None:
        """Load the MPPI YAML config from package share or an absolute path."""

        if os.path.isabs(self.mppi_config_file):
            config_path = self.mppi_config_file
        else:
            pkg_dir = get_package_share_directory("go2_exact_mppi_follow")
            config_path = os.path.join(pkg_dir, "config", "mppi_config", self.mppi_config_file)

        with open(config_path, "r", encoding="utf-8") as config_file:
            self.planner_cfg = yaml.safe_load(config_file)

        mppi_cfg = self.planner_cfg.get("MPPI", {}) or {}
        self.mppi_horizon = int(mppi_cfg.get("time_steps", 20))
        self.footprint_vertices = mppi_cfg.get(
            "vertices",
            [[[0.45, 0.23], [-0.38, 0.23], [-0.38, -0.23], [0.45, -0.23]]],
        )
        self.cloud_config = CloudFilterConfig(
            x_min=self.cloud_config.x_min,
            x_max=self.cloud_config.x_max,
            y_abs=self.cloud_config.y_abs,
            z_min=self.cloud_config.z_min,
            z_max=self.cloud_config.z_max,
            voxel_size=self.cloud_config.voxel_size,
            max_points=int(mppi_cfg.get("max_obs_num", 300)),
            corridor_width=self.cloud_config.corridor_width,
        )

    def _build_controller(self) -> None:
        """Create MPPIController and install the Go2 footprint."""

        mppi_cfg = dict(self.planner_cfg.get("MPPI", {}) or {})
        self.kinematics = str(mppi_cfg.get("motion_model", "omni"))
        self.mppi_controller = MPPIController(**mppi_cfg)
        self.mppi_controller.setRectangleFootprint(self.footprint_vertices)

    def _create_ros_interfaces(self) -> None:
        """Create all ROS subscriptions, publishers, and the control timer."""

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self._cloud_callback,
            10,
            callback_group=self.callback_group,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            10,
            callback_group=self.callback_group,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_local_topic,
            self._goal_callback,
            10,
            callback_group=self.callback_group,
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.raw_cmd_pub = self.create_publisher(Twist, self.cmd_vel_raw_topic, 10)
        self.filtered_points_pub = self.create_publisher(PointCloud2, self.filtered_points_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.cost_pub = self.create_publisher(Float32MultiArray, self.cost_breakdown_topic, 10)
        self.plan_pub = self.create_publisher(Path, self.plan_topic, 10)

        period = 1.0 / max(1.0, self.control_frequency)
        self.timer = self.create_timer(period, self._run_control, callback_group=self.control_group)

    def _cloud_callback(self, msg: PointCloud2) -> None:
        """Filter a stereo PointCloud2 message into local xy obstacle points."""

        try:
            base_points = self._pointcloud_to_base_points(msg)
            with self.state_lock:
                goal_xy = None if self.latest_goal is None else (
                    float(self.latest_goal[0]),
                    float(self.latest_goal[1]),
                )
            obstacles_xy = filter_cloud_points(
                base_points,
                self.cloud_config,
                self.footprint_vertices,
                goal_xy=goal_xy,
            )
            with self.state_lock:
                self.latest_points = obstacles_xy
                self.latest_cloud_time = self.get_clock().now()
            self.filtered_points_pub.publish(self._make_pointcloud2(obstacles_xy))
        except Exception as exc:
            self.get_logger().warn(
                f"PointCloud2 filtering failed: {exc}\n{traceback.format_exc()}",
                throttle_duration_sec=2.0,
            )

    def _pointcloud_to_base_points(self, msg: PointCloud2) -> list[tuple[float, float, float]]:
        """Read PointCloud2 xyz fields and transform them into base_frame."""

        raw_points = []
        for idx, point in enumerate(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ):
            if idx >= self.max_raw_points:
                break
            raw_points.append((float(point[0]), float(point[1]), float(point[2])))

        if not raw_points:
            return []

        arr = np.asarray(raw_points, dtype=np.float32).reshape((-1, 3))
        cloud_frame = msg.header.frame_id or self.base_frame
        if cloud_frame == self.base_frame:
            return [tuple(row) for row in arr]

        transform = self._lookup_transform(self.base_frame, cloud_frame, msg.header.stamp)
        q = transform.transform.rotation
        rotation = rotation_matrix_from_quaternion(q.x, q.y, q.z, q.w)
        translation = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )
        transformed = transform_points(arr, translation, rotation)
        return [tuple(row) for row in transformed]

    def _lookup_transform(self, target_frame: str, source_frame: str, stamp) -> object:
        """Lookup TF with latest-time fallback behavior controlled by parameters."""

        try:
            tf_time = Time() if self.use_latest_tf else Time.from_msg(stamp)
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                tf_time,
                timeout=Duration(seconds=self.transform_timeout_sec),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            # A latest transform fallback is useful when camera stamps lag TF slightly.
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self.transform_timeout_sec),
            )

    def _odom_callback(self, msg: Odometry) -> None:
        """Cache current robot velocity from odometry."""

        speed = np.array(
            [
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.linear.y),
                float(msg.twist.twist.angular.z),
            ],
            dtype=np.float32,
        )
        with self.state_lock:
            self.latest_speed = speed
            self.latest_odom_time = self.get_clock().now()
            self.have_odom = True

    def _goal_callback(self, msg: PoseStamped) -> None:
        """Cache the latest local follow goal."""

        yaw = quaternion_to_yaw(msg.pose.orientation)
        goal = np.array(
            [float(msg.pose.position.x), float(msg.pose.position.y), float(yaw)],
            dtype=np.float32,
        )
        with self.state_lock:
            self.latest_goal = goal
            self.latest_goal_time = self.get_clock().now()

    def _run_control(self) -> None:
        """Run one MPPI control tick and publish velocity commands."""

        self.tick_count += 1
        tick_start = time.perf_counter()

        if not self._inputs_ready():
            self._publish_zero()
            return

        with self.state_lock:
            latest_goal = None if self.latest_goal is None else self.latest_goal.copy()
            latest_points = None if self.latest_points is None else self.latest_points.copy()
            latest_speed = self.latest_speed.copy()

        assert latest_goal is not None
        assert latest_points is not None

        if self._nearest_obstacle_distance(latest_points) < self.emergency_stop_distance:
            self._publish_status("stop: obstacle emergency")
            self._publish_zero()
            return

        robot_pose_local = np.zeros((3,), dtype=np.float32)
        plan_local = self._build_goal_plan(latest_goal)

        try:
            # The exact_mppi controller expects robot-frame pose, speed, plan, goal, and obstacle points.
            with self.mppi_lock:
                action = self.mppi_controller.computeVelocityCommands(
                    robot_pose=robot_pose_local,
                    robot_speed=latest_speed,
                    plan=plan_local,
                    goal=latest_goal,
                    lidar_points=latest_points,
                )
        except Exception as exc:
            self.get_logger().error(f"EXACT-MPPI exception: {exc}\n{traceback.format_exc()}")
            self._publish_status("stop: mppi exception")
            self._publish_zero()
            return

        cmd = self._action_to_twist(action)
        self.raw_cmd_pub.publish(cmd)
        self.cmd_pub.publish(cmd)
        self._publish_cost_breakdown()
        self._publish_plan()
        self._publish_timing(tick_start)

    def _inputs_ready(self) -> bool:
        """Validate freshness of goal, cloud, and odometry inputs."""

        now = self.get_clock().now()
        with self.state_lock:
            goal_missing = self.latest_goal is None
            goal_age = (now - self.latest_goal_time).nanoseconds * 1e-9
            cloud_missing = self.latest_points is None
            cloud_age = (now - self.latest_cloud_time).nanoseconds * 1e-9
            odom_missing = not self.have_odom
            odom_age = (now - self.latest_odom_time).nanoseconds * 1e-9

        if goal_missing or goal_age > self.goal_timeout_sec:
            self._publish_status("stop: goal stale")
            return False
        if cloud_missing or cloud_age > self.cloud_timeout_sec:
            self._publish_status("stop: cloud stale")
            return False
        if odom_missing or odom_age > self.odom_timeout_sec:
            self._publish_status("stop: odom stale")
            return False
        return True

    def _build_goal_plan(self, goal: np.ndarray) -> np.ndarray:
        """Build a straight local reference plan from the robot to the current goal."""

        start = np.zeros((3,), dtype=np.float32)
        return np.linspace(start, goal.astype(np.float32), max(2, self.mppi_horizon)).astype(np.float32)

    def _action_to_twist(self, action) -> Twist:
        """Convert the MPPI action vector into a Go2 Twist command."""

        msg = Twist()
        if action is None:
            return msg

        values = np.asarray(action, dtype=np.float32).reshape(-1)
        if values.size < 2 or not np.all(np.isfinite(values)):
            return msg

        if self.kinematics == "omni":
            msg.linear.x = float(values[0])
            msg.linear.y = float(values[1])
            msg.angular.z = float(values[2]) if values.size >= 3 else 0.0
        elif self.kinematics == "omni_xy":
            msg.linear.x = float(values[0])
            msg.linear.y = float(values[1])
            msg.angular.z = 0.0
        else:
            msg.linear.x = float(values[0])
            msg.angular.z = float(values[1])
        return msg

    def _nearest_obstacle_distance(self, points_xy: np.ndarray) -> float:
        """Return the nearest xy obstacle distance, or infinity when no points exist."""

        if points_xy is None or points_xy.size == 0:
            return float("inf")
        distances = np.sqrt(np.einsum("ij,ij->i", points_xy, points_xy))
        return float(np.min(distances)) if distances.size else float("inf")

    def _make_pointcloud2(self, points_xy: np.ndarray) -> PointCloud2:
        """Create a base-frame PointCloud2 from selected xy obstacle points."""

        header = self.get_clock().now().to_msg()
        points = [(float(p[0]), float(p[1]), 0.0) for p in points_xy.reshape((-1, 2))]
        msg = PointCloud2()
        msg.header.stamp = header
        msg.header.frame_id = self.base_frame
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = False
        data = bytearray(msg.row_step)
        for idx, point in enumerate(points):
            struct.pack_into("fff", data, idx * msg.point_step, *point)
        msg.data = bytes(data)
        return msg

    def _publish_plan(self) -> None:
        """Publish the current optimal MPPI trajectory when available."""

        if not self.publish_plan_enabled:
            return
        try:
            traj = self.mppi_controller.getOptimalTrajectory()
        except Exception:
            return
        if traj is None:
            return

        arr = np.asarray(traj, dtype=np.float32)
        if arr.ndim != 2 or arr.shape[0] == 0 or arr.shape[1] < 2:
            return

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.base_frame
        for row in arr:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(row[0])
            pose.pose.position.y = float(row[1])
            pose.pose.orientation = yaw_to_quaternion(float(row[2]) if row.size >= 3 else 0.0)
            path.poses.append(pose)
        self.plan_pub.publish(path)

    def _publish_cost_breakdown(self) -> None:
        """Publish mean critic costs reported by the EXACT-MPPI controller."""

        if not self.publish_cost_breakdown_enabled:
            return
        try:
            costs_debug = self.mppi_controller.getCostsDebug()
        except Exception:
            return
        if not costs_debug:
            return

        payload: Dict[str, float] = {}
        for name, values in costs_debug.items():
            arr = np.asarray(values, dtype=np.float32)
            if arr.size:
                payload[str(name)] = float(np.mean(arr))
        if not payload:
            return

        keys = sorted(payload.keys())
        msg = Float32MultiArray()
        msg.data = [payload[key] for key in keys]
        msg.layout.dim = [
            MultiArrayDimension(label="keys:" + ",".join(keys), size=len(msg.data), stride=len(msg.data))
        ]
        self.cost_pub.publish(msg)

    def _publish_timing(self, tick_start: float) -> None:
        """Publish periodic timing and selected point-count status."""

        if self.tick_count % self.timing_log_every_n != 0:
            return
        elapsed_ms = (time.perf_counter() - tick_start) * 1000.0
        with self.state_lock:
            points = 0 if self.latest_points is None else int(self.latest_points.shape[0])
        self.get_logger().info(f"mppi_tick={elapsed_ms:.2f}ms selected_points={points}")
        self._publish_status(f"ok: mppi {elapsed_ms:.1f}ms points={points}")

    def _publish_zero(self) -> None:
        """Publish zero velocity to both raw and final command topics."""

        zero = Twist()
        self.raw_cmd_pub.publish(zero)
        self.cmd_pub.publish(zero)

    def _publish_status(self, text: str) -> None:
        """Publish a compact string status for launch-time debugging."""

        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None) -> None:
    """Run the Go2 EXACT-MPPI ROS2 node."""

    os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = Go2ExactMppiNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
