# Go2 UWB Smac Hybrid + MPPI 跟随避障

本包用于 Unitree Go2 的 UWB 目标跟随。当前链路是：UWB 目标点有效后，直接把 UWB 点作为 Nav2 Smac Hybrid 的规划目标，`planner_server` 先在 `odom` 下的 rolling `global_costmap` 中计算路径，再交给 `controller_server` 的 MPPI `FollowPath` 控制器跟踪。

当 UWB 目标距离小于 `follow_distance_m` 时，节点不规划、不跟踪，直接取消当前 action 并向 `/cmd_vel_nav` 发布 0 速度。

## 数据流

```text
/libAoa_robot_publisher
  -> one1000_target_point_node
  -> /uwb/target_point
  -> uwb_follow_path_node
  -> /compute_path_to_pose
  -> planner_server / SmacPlannerHybrid
  -> /uwb_follow/path
  -> /follow_path
  -> controller_server / MPPI
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel_safe

/local_grid_obstacle
  -> target_obstacle_filter_node
  -> /local_grid_obstacle_filtered
  -> global_costmap + local_costmap

/local_grid_ground
  -> global_costmap + local_costmap clearing
```

## 主要节点

### `one1000_target_point_node`

订阅 `/libAoa_robot_publisher`，输出统一目标点 `/uwb/target_point`。

有效性判断只看 `LibAoaRobotMsg.state`，默认 `state >= 0` 接收。节点不按 `pos_confidence`、RSSI、跳变距离或目标速度拒绝目标。

保留的基础检查：

- 坐标必须是有限值
- 距离必须在 `min_target_distance_m` 到 `max_target_distance_m` 之间
- 开启 `require_tf` 时，TF 转换失败会丢弃该目标

### `uwb_follow_path_node`

订阅 `/uwb/target_point`，将目标转换到 `base_footprint` 和 `odom`。

规则：

- 若 UWB 目标距离 `< follow_distance_m`，发布 0 速度并清空 `/uwb_follow/path`
- 若 UWB 目标距离 `>= follow_distance_m`，直接把 UWB 点作为 `/compute_path_to_pose` 的 goal
- Smac Hybrid 返回路径后，发布 `/uwb_follow/path`
- 随后把路径发送给 `/follow_path`，由 MPPI 跟踪
- 目标变化小于 `goal_update_distance_m` 且角度变化小于 `goal_update_angle_rad` 时，不重复规划

### `target_obstacle_filter_node`

订阅 `/uwb_follow/target_filtered` 和 `/local_grid_obstacle`，清除 UWB 目标附近一小段圆柱区域内的点云，输出 `/local_grid_obstacle_filtered`。

这样可以降低“被跟随的人本身被双目点云当成障碍，导致规划目标不可达”的概率。

## Nav2 配置

`nav2_mppi_controller.yaml` 中启动：

- `planner_server`
- `controller_server`
- `global_costmap`
- `local_costmap`
- `velocity_smoother`

全局规划器：

- 插件：`nav2_smac_planner/SmacPlannerHybrid`
- `motion_model_for_search: DUBIN`
- 不使用倒车路径

地图：

- 不使用静态大地图
- `global_costmap` 和 `local_costmap` 都是 `odom` 下 rolling window
- 两个 costmap 都使用 `/local_grid_obstacle_filtered` 和 `/local_grid_ground`

## 关键参数

配置文件：`config/uwb_mppi_follow.yaml`

- `follow_distance_m`：UWB 目标小于该距离时发布 0 速度，默认 `1.0`
- `planner_action`：默认 `/compute_path_to_pose`
- `planner_id`：默认 `GridBased`
- `planner_timeout_sec`：全局规划等待超时，默认 `1.0`
- `follow_path_action`：默认 `/follow_path`
- `controller_id`：默认 `FollowPath`
- `target_timeout_sec`：UWB 超时停车时间，默认 `1.0`
- `smoothing_alpha`：默认 `1.0`，即不平滑，直接使用 UWB 点

## 运行

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install --packages-select uwb_aoa_pkg go2_uwb_mppi_follow
source install/setup.bash
```

启动 UWB 转换、目标规划、Smac Hybrid、MPPI 和速度平滑：

```bash
ros2 launch go2_uwb_mppi_follow uwb_mppi_nav2.launch.py
```

速度桥接节点应订阅：

```text
/cmd_vel_safe
```

## 检查

```bash
ros2 node list
ros2 action list | grep compute_path_to_pose
ros2 action list | grep follow_path
ros2 topic echo /follow/uwb_path_status
ros2 topic echo /uwb_follow/path --once
ros2 topic echo /cmd_vel_safe --once
```

关键节点应包含：

- `/planner_server`
- `/controller_server`
- `/velocity_smoother`
- `/one1000_target_point_node`
- `/uwb_follow_path_node`
- `/target_obstacle_filter_node`

## 典型现象

- `state >= 0` 且 `pos_confidence = 0` 时，UWB 目标仍应被接收
- `state < 0` 时，UWB 目标会被拒绝
- UWB 目标距离 `< follow_distance_m` 时，`/cmd_vel_nav` 发布 0
- UWB 目标距离 `>= follow_distance_m` 时，请求 `/compute_path_to_pose`
- RViz 中 `/uwb_follow/path` 应来自 Smac Hybrid 规划结果
- MPPI 输出 `/cmd_vel_nav`，速度平滑后输出 `/cmd_vel_safe`

## 限制

这里的“全局规划”不是静态大地图导航，而是在 `odom` rolling `global_costmap` 中规划。目标超出 rolling 窗口、走到墙后或需要长距离绕行时，规划可能失败，节点会停车等待新的可行 UWB 目标。
