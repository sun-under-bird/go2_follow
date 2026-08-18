import math
from typing import Iterable, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from .geometry import transform_from_ros, transform_point, yaw_to_quaternion
from .grid_planner import (
    GridSpec,
    astar,
    build_costmap,
    cells_to_points,
    clear_radius,
    inflate_cells,
    nearest_free_cell,
    occupancy_grid_data,
    point_cells,
    project_goal_to_grid,
    raytrace_cells,
    resample_path,
    simplify_path,
)


class LocalPathPlanner(Node):
    def __init__(self):
        super().__init__("local_path_planner")

        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("path_frame", "odom")
        self.declare_parameter("pointcloud_topic", "/local_grid_obstacle")
        self.declare_parameter("ground_topic", "/local_grid_ground")
        self.declare_parameter("goal_topic", "/follow_goal")
        self.declare_parameter("target_valid_topic", "/follow/target_valid")
        self.declare_parameter("path_topic", "/follow_path")
        self.declare_parameter("path_valid_topic", "/follow/path_valid")
        self.declare_parameter("planner_status_topic", "/follow/planner_status")
        self.declare_parameter("costmap_topic", "/local_costmap")
        self.declare_parameter("width_m", 5.0)
        self.declare_parameter("height_m", 4.0)
        self.declare_parameter("origin_x", -0.5)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("obstacle_x_min", 0.05)
        self.declare_parameter("obstacle_x_max", 4.0)
        self.declare_parameter("obstacle_y_abs", 1.5)
        self.declare_parameter("obstacle_z_min", 0.05)
        self.declare_parameter("obstacle_z_max", 1.2)
        self.declare_parameter("inflation_radius", 0.35)
        self.declare_parameter("start_clear_radius", 0.30)
        self.declare_parameter("nearest_goal_search_radius", 0.8)
        self.declare_parameter("goal_reached_tolerance", 0.15)
        self.declare_parameter("path_spacing", 0.10)
        self.declare_parameter("obstacle_hold_sec", 0.6)
        self.declare_parameter("min_points_per_cell", 2)
        self.declare_parameter("ground_clear_enabled", True)
        self.declare_parameter("ground_timeout_sec", 0.7)
        self.declare_parameter("ground_z_min", 0.0)
        self.declare_parameter("ground_z_max", 0.25)
        self.declare_parameter("ground_clear_radius", 0.12)
        self.declare_parameter("ground_min_points_per_cell", 1)
        self.declare_parameter("ground_clear_overrides_obstacles", True)
        self.declare_parameter("ground_raytrace_clear_enabled", True)
        self.declare_parameter("goal_projection_enabled", True)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("emergency_x_max", 0.45)
        self.declare_parameter("emergency_y_abs", 0.45)
        self.declare_parameter("max_points_per_cloud", 60000)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.path_frame = str(self.get_parameter("path_frame").value)
        width_m = float(self.get_parameter("width_m").value)
        height_m = float(self.get_parameter("height_m").value)
        resolution = float(self.get_parameter("resolution").value)
        origin_x = float(self.get_parameter("origin_x").value)
        self.spec = GridSpec(
            width_m=width_m,
            height_m=height_m,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=-height_m * 0.5,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_points: List[Tuple[float, float, float]] = []
        self.latest_ground_points: List[Tuple[float, float, float]] = []
        self.latest_cloud_time = None
        self.latest_ground_time = None
        self.latest_goal: Optional[PoseStamped] = None
        self.target_valid = False
        self.held_occupied = {}
        self.last_status = ""

        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("pointcloud_topic").value),
            self._cloud_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("ground_topic").value),
            self._ground_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(PoseStamped, str(self.get_parameter("goal_topic").value), self._goal_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter("target_valid_topic").value), self._valid_cb, 10)

        self.path_pub = self.create_publisher(Path, str(self.get_parameter("path_topic").value), 10)
        self.path_valid_pub = self.create_publisher(Bool, str(self.get_parameter("path_valid_topic").value), 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, str(self.get_parameter("costmap_topic").value), 2)
        self.status_pub = self.create_publisher(String, str(self.get_parameter("planner_status_topic").value), 10)

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self._plan_and_publish)
        self.get_logger().info("local_path_planner started")

    def _read_cloud_points(self, msg: PointCloud2) -> Optional[List[Tuple[float, float, float]]]:
        transform = None
        if msg.header.frame_id and msg.header.frame_id != self.base_frame:
            try:
                transform = transform_from_ros(
                    self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time())
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"Point cloud TF failed: {exc}", throttle_duration_sec=2.0)
                return None

        points = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            px, py, pz = float(point[0]), float(point[1]), float(point[2])
            if transform is not None:
                px, py, pz = transform_point((px, py, pz), transform)
            points.append((px, py, pz))
        return points

    def _cloud_cb(self, msg: PointCloud2):
        points = self._read_cloud_points(msg)
        if points is None:
            return
        self.latest_points = points
        self.latest_cloud_time = self.get_clock().now()

    def _ground_cb(self, msg: PointCloud2):
        points = self._read_cloud_points(msg)
        if points is None:
            return
        self.latest_ground_points = points
        self.latest_ground_time = self.get_clock().now()

    def _goal_cb(self, msg: PoseStamped):
        self.latest_goal = msg

    def _valid_cb(self, msg: Bool):
        self.target_valid = bool(msg.data)

    def _ground_is_fresh(self) -> bool:
        if self.latest_ground_time is None:
            return False
        age = (self.get_clock().now() - self.latest_ground_time).nanoseconds * 1e-9
        return age <= float(self.get_parameter("ground_timeout_sec").value)

    def _goal_in_base(self) -> Optional[Tuple[float, float]]:
        if self.latest_goal is None:
            return None
        gx = float(self.latest_goal.pose.position.x)
        gy = float(self.latest_goal.pose.position.y)
        frame = self.latest_goal.header.frame_id or self.base_frame
        if frame == self.base_frame:
            return gx, gy
        try:
            tf = transform_from_ros(self.tf_buffer.lookup_transform(self.base_frame, frame, Time()))
            bx, by, _ = transform_point((gx, gy, 0.0), tf)
            return bx, by
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Goal TF failed: {exc}", throttle_duration_sec=2.0)
            return None

    def _build_path_msg(self, points_base: Iterable[Tuple[float, float]]) -> Path:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.path_frame

        points = list(points_base)
        base_to_path = None
        if self.path_frame != self.base_frame:
            try:
                base_to_path = transform_from_ros(
                    self.tf_buffer.lookup_transform(self.path_frame, self.base_frame, Time())
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f"Path TF failed, publishing in {self.base_frame}: {exc}",
                    throttle_duration_sec=2.0,
                )
                path.header.frame_id = self.base_frame

        transformed = []
        for x, y in points:
            if base_to_path is not None:
                x, y, _ = transform_point((x, y, 0.0), base_to_path)
            transformed.append((x, y))

        for idx, (x, y) in enumerate(transformed):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            if idx + 1 < len(transformed):
                nx, ny = transformed[idx + 1]
                yaw = math.atan2(ny - y, nx - x)
            elif idx > 0:
                px, py = transformed[idx - 1]
                yaw = math.atan2(y - py, x - px)
            else:
                yaw = 0.0
            pose.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(pose)
        return path

    def _publish_empty_path(self, valid: bool):
        msg = Bool()
        msg.data = valid
        self.path_valid_pub.publish(msg)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.path_frame
        self.path_pub.publish(path)

    def _publish_status(self, status: str):
        if status == self.last_status:
            return
        self.last_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(status)

    def _publish_costmap(self, occupied, inflated):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.base_frame
        grid.info.resolution = self.spec.resolution
        grid.info.width = self.spec.width_cells
        grid.info.height = self.spec.height_cells
        grid.info.origin.position.x = self.spec.origin_x
        grid.info.origin.position.y = self.spec.origin_y
        grid.info.origin.orientation.w = 1.0
        grid.data = occupancy_grid_data(self.spec, occupied, inflated)
        self.costmap_pub.publish(grid)

    def _clear_costmap(self):
        self.held_occupied.clear()
        self._publish_costmap(set(), set())

    def _ground_clear_cells(self):
        if not bool(self.get_parameter("ground_clear_enabled").value):
            return set()
        if not self._ground_is_fresh():
            return set()

        ground_points = []
        seen = 0
        max_points = int(self.get_parameter("max_points_per_cloud").value)
        x_min = float(self.get_parameter("obstacle_x_min").value)
        x_max = float(self.get_parameter("obstacle_x_max").value)
        y_abs = float(self.get_parameter("obstacle_y_abs").value)
        z_min = float(self.get_parameter("ground_z_min").value)
        z_max = float(self.get_parameter("ground_z_max").value)
        for px, py, pz in self.latest_ground_points:
            if seen >= max_points:
                break
            seen += 1
            if not (x_min <= px <= x_max):
                continue
            if abs(py) > y_abs:
                continue
            if not (z_min <= pz <= z_max):
                continue
            ground_points.append((px, py))

        clear_cells = point_cells(
            self.spec,
            self.latest_ground_points,
            x_min,
            x_max,
            y_abs,
            z_min,
            z_max,
            max_points,
            int(self.get_parameter("ground_min_points_per_cell").value),
        )
        if bool(self.get_parameter("ground_raytrace_clear_enabled").value):
            clear_cells.update(raytrace_cells(self.spec, (0.0, 0.0), ground_points))
        return inflate_cells(
            self.spec,
            clear_cells,
            float(self.get_parameter("ground_clear_radius").value),
        )

    def _held_obstacles(self, fresh_occupied, clear_cells):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        hold_sec = max(0.0, float(self.get_parameter("obstacle_hold_sec").value))
        clear_overrides = bool(self.get_parameter("ground_clear_overrides_obstacles").value)

        if clear_cells:
            if clear_overrides:
                for cell in clear_cells:
                    self.held_occupied.pop(cell, None)
            else:
                for cell in set(clear_cells) - set(fresh_occupied):
                    self.held_occupied.pop(cell, None)

        for cell in fresh_occupied:
            if clear_overrides and cell in clear_cells:
                continue
            self.held_occupied[cell] = now_sec

        if hold_sec <= 0.0:
            stale = [cell for cell in self.held_occupied if cell not in fresh_occupied]
        else:
            stale = [cell for cell, stamp in self.held_occupied.items() if now_sec - stamp > hold_sec]
        for cell in stale:
            self.held_occupied.pop(cell, None)
        return set(self.held_occupied.keys())

    def _update_costmap(self):
        result = build_costmap(
            self.spec,
            self.latest_points,
            float(self.get_parameter("obstacle_x_min").value),
            float(self.get_parameter("obstacle_x_max").value),
            float(self.get_parameter("obstacle_y_abs").value),
            float(self.get_parameter("obstacle_z_min").value),
            float(self.get_parameter("obstacle_z_max").value),
            float(self.get_parameter("inflation_radius").value),
            float(self.get_parameter("emergency_x_max").value),
            float(self.get_parameter("emergency_y_abs").value),
            int(self.get_parameter("max_points_per_cloud").value),
            int(self.get_parameter("min_points_per_cell").value),
        )
        clear_cells = self._ground_clear_cells()
        held_occupied = self._held_obstacles(result.occupied, clear_cells)
        held_inflated = inflate_cells(
            self.spec,
            held_occupied,
            float(self.get_parameter("inflation_radius").value),
        )
        blocked = clear_radius(
            self.spec,
            held_inflated,
            (0.0, 0.0),
            float(self.get_parameter("start_clear_radius").value),
        )
        self._publish_costmap(held_occupied, blocked)
        return blocked

    def _plan_and_publish(self):
        if self.latest_cloud_time is None:
            self._clear_costmap()
            self._publish_empty_path(False)
            self._publish_status("stop: waiting for obstacle cloud")
            return

        blocked = self._update_costmap()

        if not self.target_valid:
            self._publish_empty_path(False)
            self._publish_status("stop: UWB target invalid")
            return

        goal = self._goal_in_base()
        if goal is None:
            self._publish_empty_path(False)
            self._publish_status("stop: follow goal unavailable")
            return

        if math.hypot(goal[0], goal[1]) <= float(self.get_parameter("goal_reached_tolerance").value):
            path_msg = self._build_path_msg([(0.0, 0.0)])
            self.path_pub.publish(path_msg)
            valid_msg = Bool()
            valid_msg.data = True
            self.path_valid_pub.publish(valid_msg)
            self._publish_status("ok: target within follow distance")
            return

        start = self.spec.world_to_cell((0.0, 0.0))
        if start is None:
            self._publish_empty_path(False)
            self._publish_status("stop: base is outside local grid")
            return

        goal_cell = self.spec.world_to_cell(goal)
        if goal_cell is None:
            if bool(self.get_parameter("goal_projection_enabled").value):
                goal = project_goal_to_grid(self.spec, goal)
                goal_cell = self.spec.world_to_cell(goal) if goal is not None else None
            if goal_cell is None:
                self._publish_empty_path(False)
                self._publish_status("stop: follow goal outside local grid")
                return

        goal_cell = nearest_free_cell(
            self.spec,
            blocked,
            goal_cell,
            float(self.get_parameter("nearest_goal_search_radius").value),
        )
        if goal_cell is None:
            self._publish_empty_path(False)
            self._publish_status("stop: no nearby free goal cell")
            return

        cells = astar(self.spec, start, goal_cell, blocked)
        if not cells:
            self._publish_empty_path(False)
            self._publish_status("stop: no local path")
            return

        points = cells_to_points(self.spec, cells)
        points[0] = (0.0, 0.0)
        points = resample_path(simplify_path(points), float(self.get_parameter("path_spacing").value))
        path_msg = self._build_path_msg(points)
        self.path_pub.publish(path_msg)
        valid_msg = Bool()
        valid_msg.data = True
        self.path_valid_pub.publish(valid_msg)
        self._publish_status("ok")


def main(args=None):
    rclpy.init(args=args)
    node = LocalPathPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
