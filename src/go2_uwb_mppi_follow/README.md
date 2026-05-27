# Go2 UWB MPPI 无全局地图跟随避障

本包用于短距离 UWB 跟随。当前 MPPI 版本不依赖全局地图、`planner_server`、`NavigateToPose` 或全局路径规划；节点只把最新 UWB 相对目标点转换成一条短局部直线路径，并通过 Nav2 `FollowPath` action 交给 `controller_server` 的 MPPI 控制器。

双目/深度点云仍然进入 Nav2 local costmap。短路径只表达“目标在这个方向、距离还差多少”，局部避障由 MPPI 的 `ObstaclesCritic`、`CostCritic` 和 local costmap 负责；当局部窗口内无法安全通过时，机器人应减速或停车，而不是盲目追踪。

## 数据流

```text
/uwb/target_point (geometry_msgs/msg/PointStamped, base_link)
  -> uwb_follow_path_node
  -> /uwb_follow/target_filtered
  -> /uwb_follow/path
  -> /follow_path action
  -> controller_server / MPPI
  -> velocity_smoother
  -> /cmd_vel_safe

/local_grid_obstacle
/local_grid_ground
  -> local_costmap
  -> MPPI obstacle / cost critics
```

## 主要节点

### `uwb_follow_path_node`

输入：

- `/uwb/target_point`：`geometry_msgs/msg/PointStamped`
- 坐标系约定为 `base_link`，`x` 为前方，`y` 为左方，`z` 忽略
- 必要 TF：`odom -> base_link`

输出：

- `/uwb_follow/target_filtered`：滤波后的最新 UWB 目标点，`PointStamped`
- `/uwb_follow/path`：发送给 MPPI 的短直线路径，`nav_msgs/msg/Path`
- `/follow/target_valid`：当前目标是否有效，`std_msgs/msg/Bool`
- `/follow/uwb_path_status`：节点状态，`std_msgs/msg/String`
- `/follow_path`：Nav2 `nav2_msgs/action/FollowPath`

节点只使用最新滤波后的 UWB 点，不会把 UWB 历史点串成路径。

## 局部路径规则

每次使用最新有效 UWB 点 `(x, y)`：

- `r = hypot(x, y)`
- `theta = atan2(y, x)`
- 若 `r <= follow_distance_m + distance_deadband_m`，取消当前 `FollowPath` goal 并发布零速
- 否则生成从 `base_link` 原点到目标方向的短直线路径
- 终点距离为 `clamp(r - follow_distance_m, min_goal_distance_m, max_goal_distance_m)`
- 路径按 `path_resolution_m` 插值，默认 `0.05 m`
- 每个 pose 朝向设为 `theta`
- 路径通过 TF 转到 `odom` 后发布并发送给 `FollowPath`

为减少抖动，目标终点变化小于 `goal_update_distance_m` 且角度变化小于 `goal_update_angle_rad` 时，不重复发送新的 action goal。

## 关键参数

配置文件：`config/uwb_mppi_follow.yaml`

- `uwb_topic`：默认 `/uwb/target_point`
- `follow_distance_m`：默认 `1.0`
- `distance_deadband_m`：默认 `0.15`
- `min_goal_distance_m`：默认 `0.25`
- `max_goal_distance_m`：默认 `2.0`
- `path_resolution_m`：默认 `0.05`
- `target_timeout_sec`：默认 `0.4`
- `max_target_jump_m`：默认 `0.7`
- `publish_rate_hz`：默认 `5.0`
- `slow_turn_angle_rad`：默认约 `60 deg`，目标角度更大时缩短目标路径，让 MPPI 优先转向

## Nav2 组件

默认启动文件只启动局部导航链路：

- `controller_server`
- `local_costmap`
- `velocity_smoother`
- `lifecycle_manager`
- `uwb_follow_path_node`

不启动：

- `map_server`
- `amcl`
- `planner_server`
- `global_costmap`

local costmap 使用：

- `global_frame: odom`
- `robot_base_frame: base_link`
- `rolling_window: true`
- 障碍点云：`/local_grid_obstacle`
- 地面/清障点云：`/local_grid_ground`

## 运行

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install --packages-select go2_uwb_mppi_follow
source install/setup.bash
```

只启动 UWB 路径节点：

```bash
ros2 launch go2_uwb_mppi_follow uwb_mppi_follow.launch.py
```

启动 UWB 路径节点和 Nav2 MPPI 局部控制链路：

```bash
ros2 launch go2_uwb_mppi_follow uwb_mppi_nav2.launch.py
```

## 测试建议

发布测试目标点：

```bash
ros2 topic pub /uwb/target_point geometry_msgs/msg/PointStamped "{header: {frame_id: base_link}, point: {x: 2.0, y: 0.0, z: 0.0}}"
```

检查：

- `/uwb_follow/target_filtered`
- `/uwb_follow/path`
- `/transformed_global_plan`
- local costmap 障碍层
- `/cmd_vel_safe`

典型现象：

- UWB 为 `(2.0, 0.0)` 时，路径终点约在前方 `1.0 m`
- UWB 为 `(1.05, 0.0)` 时，进入跟随距离死区并停车
- UWB 为 `(1.5, 0.8)` 时，生成斜前方短路径
- UWB 断开约 `0.4 s` 后取消 `FollowPath` 并停车
- 前方出现障碍后，MPPI 应小范围绕行、减速或停车

## 限制

本方案只保证局部窗口内避障。目标跑到墙后、跨房间、或需要绕远路时，机器人应停住等待新的可行局部目标，不会做全局绕行追踪。
