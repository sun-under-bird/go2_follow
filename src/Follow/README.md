# Go2 UWB 动态跟随避障

本包用于 Unitree Go2 上的低速 UWB 跟随避障，ROS2 版本按 Humble 设计。当前默认架构已经改为参考 Nav2 的动态目标跟随教程：UWB 负责持续发布动态目标，Nav2 负责基于 costmap 规划和 MPPI 控制，本包最后用 `safety_mux` 做硬安全门控。

## 默认链路

```text
/libAoa_robot_publisher
  -> follow_goal_node
  -> /one1000/target            # 调试用，base_link 下的滤波目标
  -> /goal_update               # 给 Nav2 GoalUpdater 的动态目标，odom 下
  -> /follow/target_valid

Nav2 bt_navigator:
  GoalUpdater
  -> ComputePathToPose
  -> TruncatePath(distance=1.5)
  -> FollowPath / MPPI
  -> /cmd_vel_nav

safety_mux
  -> /cmd_vel_safe
  -> go2_twist_bridge
```

旧的 `local_path_planner + follow_path_action_client` 仍然保留作备用，但默认不再启动。默认走 `behavior_trees/uwb_dynamic_follow.xml`。

## 依赖的 Go2 链路

参考 `C:\Users\chy\go2`：

- 相机：`stereo_camera_pkg/launch/usb_400.launch.py`
- 里程计：`go2_driver` 发布 `/odom_leg`
- 障碍点云：RTAB-Map 发布 `/local_grid_obstacle`
- 地面清除点云：RTAB-Map 发布 `/local_grid_ground`
- 速度执行：`go2_twist_bridge`，必须重映射为订阅 `/cmd_vel_safe`

## 必要 TF

必须有：

```text
odom -> base_footprint -> base_link -> camera_link -> camera_left_frame
                                             -> camera_right_frame
```

- `odom -> base_footprint`：由 `go2_driver` 发布。
- `base_footprint -> base_link`：由 Go2 URDF / `robot_state_publisher` 发布。
- 相机 TF：由 Go2 描述或 RTAB-Map launch 发布。
- UWB 当前默认输出 `base_link` 下的 `x/y`，`follow_goal_node` 会再转成 `odom` 下的 `/goal_update` 给 Nav2。

## 主要话题

输入：

- `/libAoa_robot_publisher`：UWB 数据，类型 `uwb_aoa_pkg/msg/LibAoaRobotMsg`
- `/odom_leg`：足式里程计
- `/local_grid_obstacle`：RTAB-Map 障碍点云
- `/local_grid_ground`：RTAB-Map 地面点云
- `/cmd_vel_nav`：Nav2 Controller 输出速度

输出：

- `/one1000/target`：滤波后的 UWB 目标
- `/follow_goal`：兼容旧链路的保持距离目标
- `/goal_update`：Nav2 `GoalUpdater` 订阅的动态目标
- `/cmd_vel_safe`：安全门控后的最终速度
- `/follow/target_status`：UWB 状态
- `/follow/nav2_status`：Nav2 动态跟随 action 状态
- `/follow/safety_status`：安全门控状态

## 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow_ws
colcon build --symlink-install
source install/setup.bash
```

每个终端都需要 source Go2 工作空间和本工作空间。

## 推荐启动顺序

先启动 Go2 TF 和里程计：

```bash
ros2 launch go2_description display.launch.py
ros2 run go2_driver driver
```

检查：

```bash
ros2 topic hz /odom_leg
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
```

启动 UWB：

```bash
ros2 run uwb_aoa_pkg libAoa_robot_example /dev/ttyUSB0
ros2 topic hz /libAoa_robot_publisher
```

启动相机和 RTAB-Map：

```bash
ros2 launch stereo_camera_pkg usb_400.launch.py
ros2 launch stereo_camera_pkg navigation.launch.py use_nav2:=false use_viz:=false
```

检查：

```bash
ros2 topic hz /local_grid_obstacle
ros2 topic hz /local_grid_ground
```

启动本包，先不要接 Go2 速度桥：

```bash
ros2 launch go2_dynamic_follow_avoidance go2_follow_bringup.launch.py \
  start_nav2_controller:=true \
  start_follow:=true \
  start_twist_bridge:=false \
  start_uwb:=false \
  start_go2_driver:=false \
  start_camera:=false \
  start_rtabmap:=false
```

观察：

```bash
ros2 topic echo /follow/target_status
ros2 topic echo /goal_update
ros2 topic echo /follow/nav2_status
ros2 topic echo /follow/safety_status
ros2 topic echo /cmd_vel_safe
```

确认架空和低速安全后，再启动桥接：

```bash
ros2 launch go2_dynamic_follow_avoidance go2_follow_bringup.launch.py \
  start_twist_bridge:=true \
  start_nav2_controller:=false \
  start_follow:=false
```

这个 launch 会把 `go2_twist_bridge` 的 `cmd_vel` 重映射到 `/cmd_vel_safe`。

## 无相机简单跟随

如果暂时不使用相机和 RTAB-Map，只想用 UWB 让 Go2 低速跟随，可以启动简单模式。这个模式没有避障，只做 UWB 距离跟随和朝向调整，仍然经过 `safety_mux` 输出 `/cmd_vel_safe`。

先启动 Go2 TF、里程计和 UWB，然后运行：

```bash
ros2 launch go2_dynamic_follow_avoidance go2_follow_bringup.launch.py \
  start_nav2_controller:=false \
  start_follow:=true \
  use_simple_follow:=true \
  require_pointcloud_watchdog:=false \
  start_twist_bridge:=false
```

先观察：

```bash
ros2 topic echo /follow/target_status
ros2 topic echo /one1000/target
ros2 topic echo /follow/simple_status
ros2 topic echo /follow/safety_status
ros2 topic echo /cmd_vel_safe
```

确认 `/cmd_vel_safe` 符合预期后，再启动安全桥接：

```bash
ros2 launch go2_dynamic_follow_avoidance go2_follow_bringup.launch.py \
  start_twist_bridge:=true \
  start_nav2_controller:=false \
  start_follow:=false
```

无相机模式下没有障碍物感知，前方有人或物体不会自动绕开或急停。只建议架空测试、空旷低速测试，遥控器必须随时能接管。

## 一键启动

第一次不建议一键启动。联调稳定后可以使用：

```bash
ros2 launch go2_dynamic_follow_avoidance go2_follow_bringup.launch.py \
  start_go2_driver:=true \
  start_twist_bridge:=true \
  start_camera:=true \
  start_rtabmap:=true \
  start_uwb:=true \
  start_nav2_controller:=true \
  start_follow:=true \
  uwb_device:=/dev/ttyUSB0 \
  uwb_frame_id:=base_link
```

## 关键参数

- `follow_goal_node.nav2_goal_topic: /goal_update`
- `follow_goal_node.nav2_goal_frame: odom`
- `follow_goal_node.follow_distance: 1.5`
- `safety_mux.cmd_vel_in: /cmd_vel_nav`
- `safety_mux.cmd_vel_out: /cmd_vel_safe`
- `safety_mux.odom_topic: /odom_leg`
- `safety_mux.require_path_watchdog: false`
- `controller_server.odom_topic: /odom_leg`
- `bt_navigator.goal_updater_topic: /goal_update`

`safety_mux` 默认不再依赖旧的 `/follow_path`，因为路径由 Nav2 BT 内部 action 管理。最后安全仍然检查 UWB、`/odom_leg`、RTAB-Map 点云、Nav2 速度和前方障碍。

## 安全边界

默认 `max_vx=0.2`，前方 `0.45m` 内障碍急停，`1.0m` 内慢行。第一次实测必须架空或低速，遥控器随时准备接管。任何关键输入超时都会让 `/cmd_vel_safe` 归零。
