# Go2 UWB 跟随避障

本包面向 Unitree Go2 的低速 UWB 跟随避障。硬件链路参考 `C:\Users\chy\go2`：

- 相机启动参考 `stereo_camera_pkg/launch/usb_400.launch.py`
- 里程计使用 `go2_driver` 发布的 `/odom_leg`
- 障碍点云使用 RTAB-Map 输出的 `/local_grid_obstacle`
- 地面清除点云使用 `/local_grid_ground`
- 最终只允许 `/cmd_vel_safe` 进入 `go2_twist_bridge`

默认策略是宁停不误动。UWB、里程计、RTAB-Map 点云、局部路径、Nav2 速度任一链路超时或失效，`safety_mux` 都会立即输出零速度。

## 控制链路

```text
/libAoa_robot_publisher
  -> follow_goal_node
  -> /one1000/target, /follow_goal, /follow/target_valid
  -> local_path_planner
  -> /follow_path, /follow/path_valid
  -> Nav2 controller_server / MPPI
  -> /cmd_vel_nav
  -> safety_mux
  -> /cmd_vel_safe
  -> go2_twist_bridge
```

不要让任何节点直接向 Go2 桥接的 `/cmd_vel` 发速度。`go2_twist_bridge` 必须重映射为订阅 `/cmd_vel_safe`。

## 必要 TF

实际 TF 链应为：

```text
odom -> base_footprint -> base_link -> camera_link -> camera_left_frame
                                             -> camera_right_frame
```

- `odom -> base_footprint` 由 `go2_driver` 发布。
- `base_footprint -> base_link` 由 Go2 URDF 和 `robot_state_publisher` 发布。
- 相机 TF 由 Go2 描述或 RTAB-Map launch 发布。
- UWB 默认按 `base_link` 下的目标坐标处理，匹配本仓库里的 `uwb_aoa_pkg` 发布器。若你的 UWB 锚点是独立坐标系，可以把 `one1000_frame` 改成锚点 frame，并提供到 `base_link` 的 TF。

## 主要话题

输入：

- `/libAoa_robot_publisher`：UWB 标签位置，类型 `uwb_aoa_pkg/msg/LibAoaRobotMsg`
- `/odom_leg`：足式里程计
- `/local_grid_obstacle`：RTAB-Map 障碍点云
- `/local_grid_ground`：RTAB-Map 地面清除点云
- `/cmd_vel_nav`：Nav2 Controller Server 输出速度

输出：

- `/one1000/target`：滤波后的 UWB 目标
- `/follow_goal`：保持跟随距离后的局部目标
- `/follow_path`：给 Nav2 FollowPath 的短路径
- `/cmd_vel_safe`：安全门控后的最终速度
- `/follow/target_status`：UWB 目标状态
- `/follow/planner_status`：局部规划状态
- `/follow/safety_status`：安全门控状态

## 编译

把本仓库放在 ROS2 工作空间中：

```bash
cd ~/go2_follow_ws
colcon build --symlink-install
source install/setup.bash
```

运行前还需要 source `C:\Users\chy\go2` 对应工作空间在机器人上的 ROS2 install 环境，使 `go2_driver`、`go2_twist_bridge`、`stereo_camera_pkg`、RTAB-Map 相关包可见。

## 推荐启动顺序

先单独确认 Go2 驱动：

```bash
ros2 launch go2_driver driver.launch.py
ros2 topic echo /odom_leg
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
```

然后启动相机和 RTAB-Map，确认点云：

```bash
ros2 launch stereo_camera_pkg usb_400.launch.py
ros2 launch stereo_camera_pkg navigation.launch.py use_nav2:=false use_viz:=false
ros2 topic hz /local_grid_obstacle
ros2 topic hz /local_grid_ground
```

最后启动本包：

```bash
ros2 launch go2_dynamic_follow_avoidance go2_follow_bringup.launch.py \
  start_nav2_controller:=true \
  start_follow:=true
```

如果希望本包一起拉起相机、RTAB-Map、UWB、Go2 节点，可以使用：

```bash
ros2 launch go2_dynamic_follow_avoidance go2_follow_bringup.launch.py \
  start_go2_driver:=true \
  start_twist_bridge:=true \
  start_camera:=true \
  start_rtabmap:=true \
  start_uwb:=true \
  uwb_device:=/dev/ttyUSB0 \
  uwb_frame_id:=base_link
```

这里的 `start_twist_bridge:=true` 会启动同一个 `go2_twist_bridge`，但已经重映射为 `cmd_vel:=/cmd_vel_safe`。

## 关键参数

默认参数在 `config/go2_dynamic_follow_avoidance.yaml`：

- `follow_goal_node` 不订阅里程计，跟随目标只依赖 UWB 和必要 TF。
- `local_path_planner.pointcloud_topic: /local_grid_obstacle`
- `safety_mux.odom_topic: /odom_leg`
- `safety_mux.cmd_vel_in: /cmd_vel_nav`
- `safety_mux.cmd_vel_out: /cmd_vel_safe`
- `safety_mux.max_vx: 0.2`
- `safety_mux.emergency_x_max: 0.45`
- `safety_mux.obstacle_clear_x_max: 0.60`

UWB 目标滤波包含连续有效计数、置信度、跳变拒绝、EMA 平滑和短时坏样本保持。RTAB-Map 点云避障包含每栅格最小点数确认和障碍短时保持，避免点云闪烁导致路径抖动。

## 低速实测

1. 架空 Go2，确认 `/cmd_vel_safe` 默认是零。
2. 拿 UWB 标签缓慢移动，观察 `/one1000/target`、`/follow_goal` 和 `/follow/target_status`。
3. 在机器人前方放置障碍，观察 `/follow_path` 是否绕开，`/follow/safety_status` 是否进入慢行或急停。
4. 地面测试保持 `max_vx: 0.2`，遥控器随时准备接管。
5. 确认急停、点云遮挡、UWB 丢包、关闭 Nav2 任一种情况都会让 `/cmd_vel_safe` 归零。

## 说明

跟随避障需要 `/odom_leg`。Nav2 Controller 和局部路径输出在 `odom` 坐标系下工作，`safety_mux` 也用 `/odom_leg` 作为关键输入看门狗。这里使用的是 `go2_driver` 发布的足式里程计，不需要额外的轮式里程计。
