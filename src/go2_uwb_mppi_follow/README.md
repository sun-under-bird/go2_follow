# Go2 UWB MPPI 直线路径跟随避障

该包把外接 UWB 目标转换为 `base_footprint` 下的标准点，计算保持距离站位点，再在 `odom` 中生成机器人到站位点的直线路径。MPPI 在局部代价地图上采样控制序列实现短距离绕障；它不启动全局规划器。

## 数据流

```text
/libAoa_robot_publisher
  -> one1000_target_point_node
  -> /uwb/target_point
  -> uwb_follow_path_node
       /uwb_follow/target_raw
       /uwb_follow/follow_goal
       /uwb_follow/path
  -> follow_path_recovery_bt_node
       FollowPath 失败 -> BackUpTwzFree -> 重试
  -> controller_server / MPPI
  -> /cmd_vel

/local_grid_obstacle
  -> target_obstacle_filter_node
  -> /local_grid_obstacle_filtered
  -> STVL local_costmap
/local_grid_ground -> local_costmap clearing
```

`uwb_follow_path_node` 负责持续发布路径和停车空路径，不再直接发送 `FollowPath` action，也不与 controller 并发插入停车零速度；仅在启用近距离原地转向时直接发布角速度。行为树执行器只在路径终点变化超过阈值时更新 action，避免时间戳变化导致高频抢占。

## 关键行为

- 默认保持距离 `1.5 m`；近于该距离时发布空路径并停车。
- `rotate_to_target_within_follow_distance=false`，默认不会绕过 costmap 直接原地旋转。
- 直线路径按 `0.20 m` 插值，路径本身不绕障，实际局部避障由 MPPI 的 `ObstaclesCritic` 完成。
- `/uwb_follow/target_raw` 是未经低通、跳变、速度、置信度和超时门控的人员点；目标点云过滤器会清除人员附近的小圆柱，减少把被跟随者当成不可达障碍的概率。
- `FollowPath` 因碰撞、无有效控制或进度超时失败后，行为树调用空闲区域恢复并自动重试，默认两次。

## 编译与启动

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install --packages-up-to go2_uwb_mppi_follow
source install/setup.bash

# 先提供 D435i 局部障碍/地面点云
ros2 launch go2_dynamic_follow_avoidance d435i_rtabmap.launch.py

# 再启动 UWB、MPPI、behavior server 和恢复行为树
ros2 launch go2_uwb_mppi_follow uwb_mppi_nav2.launch.py
```

完整 launch 默认启动外接 UWB 驱动；已有 UWB 发布者或回放 rosbag 时使用 `start_uwb:=false`。串口可通过 `uwb_device:=...` 覆盖。

## 检查

```bash
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
ros2 topic echo /follow/uwb_path_status
ros2 topic echo /follow/mppi_recovery_status
ros2 topic echo /uwb_follow/follow_goal --once
ros2 topic echo /uwb_follow/path --once
ros2 topic hz /local_grid_obstacle_filtered
ros2 topic info /cmd_vel --verbose
```

关键节点应包括 `/controller_server`、`/behavior_server`、`/mppi_follow_recovery_bt`、`/one1000_target_point_node`、`/uwb_follow_path_node` 和 `/target_obstacle_filter_node`。项目不再启动 `velocity_smoother`，所有速度直接发到 `/cmd_vel`。

## 边界

- MPPI 只在约 2 秒预测窗内采样，不提供长距离全局绕路。
- 目标人体清除半径过大会擦除附近真实障碍，必须结合 UWB 误差实测。
- STVL 是额外运行依赖；插件缺失时 local costmap 无法配置。
- 按当前阶段要求没有安全门控或速度平滑，一次只能启动一个控制方案。
- 收到第一帧目标后会持续复用最后一个原始目标；UWB 停更不会自动停车，实机测试时必须人工接管。
