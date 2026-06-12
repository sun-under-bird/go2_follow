# Go2 UWB MPPI 直连跟随避障

本包用于 Unitree Go2 的 UWB 目标跟随。当前链路是：UWB 目标点有效且距离大于或等于 `follow_distance_m` 后，节点先计算保持跟随距离后的站位点，再生成从机器人当前位置到站位点的局部直线路径，最后交给 `controller_server` 的 MPPI `FollowPath` 控制器跟踪和避障。

当 UWB 目标距离小于 `follow_distance_m` 时，节点不使用 MPPI，直接取消当前 action，清空路径，并向速度话题发布线速度为 0、角速度朝向 UWB 目标的简单转向命令。

## 数据流

```text
/libAoa_robot_publisher
  -> one1000_target_point_node
  -> /uwb/target_point
  -> uwb_follow_path_node
  -> /uwb_follow/follow_goal
  -> /uwb_follow/path
  -> /follow_path
  -> controller_server / MPPI
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel_safe

/image_raw
  -> go2_stereo_camera / stereo_split_node
  -> stereo_image_proc
  -> /stereo/points2
  -> stereo_cloud_filter_node
  -> /local_grid_obstacle
  -> /local_grid_ground

/local_grid_obstacle
  -> target_obstacle_filter_node
  -> /local_grid_obstacle_filtered
  -> local_costmap

/local_grid_ground
  -> local_costmap clearing
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

订阅 `/uwb/target_point`，将目标转换到机器人本体坐标系和 `odom`。

规则：

- 若 UWB 目标距离 `< follow_distance_m`，取消 MPPI，发布线速度为 0 的原地转向命令并清空 `/uwb_follow/path`
- 若 UWB 目标距离 `>= follow_distance_m`，先计算保持距离后的站位点 `/uwb_follow/follow_goal`，再生成到站位点的局部直线路径并发布 `/uwb_follow/path`
- 随后把路径发送给 `/follow_path`，由 MPPI 跟踪并通过 local costmap 避障
- 目标变化小于 `goal_update_distance_m` 且角度变化小于 `goal_update_angle_rad` 时，不重复规划
- `/uwb_follow/target_filtered` 始终表示真实人员位置，供点云过滤清除“目标人本身”的障碍点

### `target_obstacle_filter_node`

订阅 `/uwb_follow/target_filtered` 和 `/local_grid_obstacle`，清除 UWB 目标附近一小段圆柱区域内的点云，输出 `/local_grid_obstacle_filtered`。

这样可以降低“被跟随的人本身被双目点云当成障碍，导致规划目标不可达”的概率。

### `stereo_cloud_filter_node`

订阅 `/stereo/points2`，转换到 `base_footprint` 后按前视 ROI、地面高度和障碍高度拆分点云：

- `/local_grid_obstacle`：供 STVL 标记障碍
- `/local_grid_ground`：供 STVL 清除地面方向空间
- 默认体素大小 `0.05m`，降低双目点云密度和 costmap 压力

## Nav2 配置

`nav2_mppi_controller.yaml` 中实际启动：

- `controller_server`
- `local_costmap`
- `velocity_smoother`

地图：

- 不使用静态大地图
- 不启动 `planner_server`，不使用 `global_costmap` 做 Smac 全局规划
- `local_costmap` 是 `odom` 下 rolling window
- `local_costmap` 使用 `/local_grid_obstacle_filtered` 和 `/local_grid_ground`

## 关键参数

配置文件：`config/uwb_mppi_follow.yaml`

- `follow_distance_m`：UWB 目标小于该距离时进入近距离原地转向模式，默认 `1.5`
- `rotate_to_target_within_follow_distance`：近距离时是否发布原地转向速度，默认 `true`
- `hold_rotate_yaw_gain`：近距离转向角速度比例增益，默认 `1.0`
- `hold_rotate_max_angular_vel`：近距离转向最大角速度，默认 `0.8`
- `hold_rotate_yaw_deadband_rad`：近距离转向角度死区，默认 `0.05`
- `direct_path_resolution_m`：直线路径插值分辨率，默认 `0.20`
- `follow_path_action`：默认 `/follow_path`
- `controller_id`：默认 `FollowPath`
- `target_timeout_sec`：UWB 超时停车时间，默认 `1.0`
- `smoothing_alpha`：默认 `0.35`，对 UWB 目标做一阶低通
- `max_target_jump_m` / `max_target_speed_mps`：拒绝明显跳变和速度尖峰
- `follow_goal_topic`：默认 `/uwb_follow/follow_goal`，表示保持跟随距离后的 MPPI 站位目标

## 运行

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install --packages-select uwb_aoa_pkg go2_stereo_camera go2_uwb_mppi_follow
source install/setup.bash
```

启动 UWB 转换、目标直线路径、MPPI 和速度平滑：

```bash
ros2 launch go2_uwb_mppi_follow uwb_mppi_nav2.launch.py
```

如果使用 1280x480 左右拼接双目相机，同时启动相机链路：

```bash
ros2 launch go2_uwb_mppi_follow uwb_mppi_nav2.launch.py \
  start_stereo_camera:=true \
  video_device:=/dev/video0 \
  left_info_url:=/path/to/left.yaml \
  right_info_url:=/path/to/right.yaml
```

速度桥接节点应订阅：

```text
/cmd_vel_safe
```

## 检查

```bash
ros2 node list
ros2 action list | grep follow_path
ros2 topic echo /follow/uwb_path_status
ros2 topic echo /uwb_follow/follow_goal --once
ros2 topic echo /uwb_follow/path --once
ros2 topic hz /stereo/points2
ros2 topic hz /local_grid_obstacle
ros2 topic hz /local_grid_ground
ros2 topic echo /cmd_vel_safe --once
```

关键节点应包含：

- `/controller_server`
- `/velocity_smoother`
- `/one1000_target_point_node`
- `/uwb_follow_path_node`
- `/target_obstacle_filter_node`

## 典型现象

- `state >= 0` 且 `pos_confidence = 0` 时，UWB 目标仍应被接收
- `state < 0` 时，UWB 目标会被拒绝
- UWB 目标距离 `< follow_distance_m` 时，不使用 MPPI，线速度为 0，只发布朝向目标的角速度
- UWB 目标距离 `>= follow_distance_m` 时，生成到 `/uwb_follow/follow_goal` 的局部直线路径并发送 `/follow_path`
- RViz 中 `/uwb_follow/path` 应来自 `uwb_follow_path_node` 生成的局部直线路径
- MPPI 输出 `/cmd_vel_nav`，速度平滑后输出 `/cmd_vel_safe`

## 限制

这里不再做全局绕行规划。路径只是朝向 UWB 点的局部参考线，障碍绕行完全依赖 MPPI 的采样轨迹和 `local_costmap` 代价函数；如果障碍物完全挡住目标方向，MPPI 可能会减速、停住或在局部范围内寻找可行绕行，但不会像全局规划器那样主动规划长距离绕路。
