# Go2 UWB 局部跟随避障

本包用于 Unitree Go2 上的低速 UWB 跟随避障，ROS2 版本按 Humble 设计。当前只保留旧版链路：本包自己用点云构建局部栅格并用 A* 规划 `/follow_path`，Nav2 只使用 `controller_server` 里的 MPPI 控制器跟踪路径，最后通过 `safety_mux` 输出 `/cmd_vel_safe`。

## 运行链路

```text
/libAoa_robot_publisher
  -> follow_goal_node
  -> /one1000/target
  -> /follow_goal
  -> /follow/target_valid

/local_grid_obstacle
  -> local_path_planner
  -> 自写局部栅格 + 障碍膨胀 + A*
  -> /local_costmap
  -> /follow_path
  -> /follow/path_valid

follow_path_action_client
  -> Nav2 follow_path action
  -> controller_server / FollowPath(MPPI)
  -> /cmd_vel_nav

safety_mux
  -> /cmd_vel_safe
  -> go2_twist_bridge
  -> Go2
```

本包只负责局部目标、局部路径和速度门控；全局导航节点不参与当前链路。

## 依赖链路

参考 `C:\Users\chy\go2`：

- 里程计：`go2_driver` 发布 `/odom_leg`
- 相机：`stereo_camera_pkg/launch/usb_400.launch.py`，或使用本包的 D435i RTAB-Map launch
- 障碍点云：RTAB-Map 发布 `/local_grid_obstacle`
- 地面清除点云：RTAB-Map 发布 `/local_grid_ground`
- 速度执行：`go2_twist_bridge`，必须重映射为订阅 `/cmd_vel_safe`

## 必要 TF

必须有：

```text
odom -> base_footprint -> base_link -> camera_link
base_link -> uwb_link
```

- `odom -> base_footprint`：由 `go2_driver` 发布。
- `base_footprint -> base_link`：由 Go2 URDF / `robot_state_publisher` 发布。
- `base_link -> uwb_link`：按 UWB 模块安装位置发布。
- 点云 frame 必须能通过 TF 转到 `base_link`。

## 主要话题

输入：

- `/libAoa_robot_publisher`：UWB 数据，类型 `uwb_aoa_pkg/msg/LibAoaRobotMsg`
- `/odom_leg`：足式里程计
- `/local_grid_obstacle`：RTAB-Map 障碍点云
- `/local_grid_ground`：RTAB-Map 地面点云，供 Nav2 local costmap 清除使用

输出：

- `/one1000/target`：滤波后的 UWB 目标，`base_link` 下
- `/follow_goal`：保持跟随距离后的局部目标
- `/local_costmap`：本包自写局部栅格地图，可在 RViz2 中显示
- `/follow_path`：本包自写 A* 局部路径
- `/cmd_vel_nav`：Nav2 MPPI 输出速度
- `/cmd_vel_safe`：安全门控后的最终速度
- `/follow/target_status`：UWB 状态
- `/follow/planner_status`：本包 A* 规划状态
- `/follow/safety_status`：安全门控状态

## 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install
source install/setup.bash
```

每个终端都需要 source Go2 工作空间和本工作空间。

## 启动顺序

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
ros2 run tf2_ros tf2_echo base_link uwb_link
```

启动 UWB：

```bash
ros2 run uwb_aoa_pkg libAoa_robot_example /dev/ttyUSB0
ros2 topic hz /libAoa_robot_publisher
```

`/libAoa_robot_publisher` 默认按 `1Hz` 发布。需要临时调整频率时：

```bash
ros2 run uwb_aoa_pkg libAoa_robot_example /dev/ttyUSB0 --ros-args -p publish_rate_hz:=1.0
```

启动相机和 RTAB-Map：

```bash
ros2 launch stereo_camera_pkg usb_400.launch.py
ros2 launch stereo_camera_pkg navigation.launch.py use_nav2:=false use_viz:=false
```

如果使用 D435i 红外双目话题测试：

```bash
ros2 launch go2_dynamic_follow_avoidance d435i_rtabmap.launch.py \
  base_frame:=base_footprint \
  odom_topic:=/odom_leg \
  use_viz:=false
```

确认点云：

```bash
ros2 topic hz /local_grid_obstacle
ros2 topic hz /local_grid_ground
```

启动本包，第一次先不要接速度桥：

```bash
ros2 launch go2_dynamic_follow_avoidance go2_follow_bringup.launch.py \
  start_nav2_controller:=true \
  start_follow:=true \
  use_simple_follow:=false \
  start_twist_bridge:=false \
  start_uwb:=false \
  start_go2_driver:=false \
  start_camera:=false \
  start_rtabmap:=false
```

观察：

```bash
ros2 topic echo /follow/target_status
ros2 topic echo /follow/planner_status
ros2 topic echo /follow_path
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel_safe
```

确认 `/cmd_vel_safe` 正常后，再单独启动桥接：

```bash
ros2 launch go2_dynamic_follow_avoidance go2_follow_bringup.launch.py \
  start_twist_bridge:=true \
  start_nav2_controller:=false \
  start_follow:=false
```

## 一键启动

联调稳定后可以使用：

```bash
ros2 launch go2_dynamic_follow_avoidance go2_follow_bringup.launch.py \
  start_go2_driver:=true \
  start_twist_bridge:=true \
  start_camera:=true \
  start_rtabmap:=true \
  start_uwb:=true \
  start_nav2_controller:=true \
  start_follow:=true \
  use_simple_follow:=false \
  uwb_device:=/dev/ttyUSB0 \
  uwb_frame_id:=uwb_link \
  uwb_publish_rate_hz:=1.0
```

## 无相机简单跟随

如果暂时不使用相机和 RTAB-Map，只想让 Go2 按 UWB 低速跟随，可以启动简单模式。这个模式没有路径避障，只做 UWB 距离和朝向控制。

```bash
ros2 launch go2_dynamic_follow_avoidance go2_follow_bringup.launch.py \
  start_nav2_controller:=false \
  start_follow:=true \
  use_simple_follow:=true \
  require_pointcloud_watchdog:=false \
  start_twist_bridge:=false
```

## RViz2 可视化

本包自写局部栅格：

```text
/local_costmap
```

在 RViz2 里添加 `Map`，Topic 选 `/local_costmap`。其中：

- `100`：原始障碍
- `60`：本包膨胀区域
- `0`：空闲区域

Nav2 MPPI 使用的 local costmap：

```text
/local_costmap/costmap
```

本包 A* 使用 `/local_costmap`，MPPI 使用 Nav2 自己的 `/local_costmap/costmap`。两者都来自 `/local_grid_obstacle`，建议保持本包膨胀半径和 Nav2 膨胀半径接近。

## 关键参数

- `follow_goal_node.one1000_frame: uwb_link`
- `follow_goal_node.follow_distance: 2.0`
- `follow_goal_node.stop_distance: 2.0`
- `local_path_planner.path_topic: /follow_path`
- `local_path_planner.inflation_radius: 0.35`
- `follow_path_action_client.action_name: follow_path`
- `follow_path_action_client.controller_id: FollowPath`
- `controller_server.FollowPath.plugin: nav2_mppi_controller::MPPIController`
- `controller_server.FollowPath.vx_max: 0.5`
- `controller_server.FollowPath.vx_min: -0.4`
- `controller_server.FollowPath.wz_max: 1.2`
- `local_costmap.local_costmap.plugins: [voxel_layer, inflation_layer]`
- `local_costmap.local_costmap.inflation_layer.inflation_radius: 0.4`
- `safety_mux.cmd_vel_in: /cmd_vel_nav`
- `safety_mux.cmd_vel_out: /cmd_vel_safe`
- `safety_mux.odom_topic: /odom_leg`

## 安全边界

当前默认 `max_vx=0.5`。第一次实测必须架空或低速，遥控器随时准备接管。确认 `/cmd_vel_safe` 的方向和速度正确以后，再启动 `go2_twist_bridge`。
