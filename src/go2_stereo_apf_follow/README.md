# Go2 双目 APF 跟随避障

这个包用 C++ 把 `jie_deamon` 的跟随避障结构搬到双目点云上。默认运行方式已经改成和参考仓库一致：不维护局部地图，不启动独立 safety mux，控制节点直接根据当前帧障碍点和当前目标发布 `/cmd_vel`。

## 方法链路

```text
/libAoa_robot_publisher
  -> uwb_target_seed_node
  -> /stereo_apf/seed_target

/local_grid_obstacle
  -> stereo_apf_controller_node
  -> 当前帧点云过滤
  -> 直接使用最新原始 UWB 目标
  -> APF 急停/减速/排斥
  -> /cmd_vel
```

本包没有独立安全门控。急停在 APF 控制内部完成：最近障碍距离小于 `0.35 m` 时发布零速度；障碍离开后下一帧自动恢复。

UWB 节点不检查 `state`、置信度、距离、跳变或超时，APF/VFH 也不做目标低通；收到第一帧后持续使用最后一个原始目标。VFH 速度不经过低通或变化率限制。

## VFH 绕障链路

本包另外提供 `stereo_vfh_controller_node`，用于 UWB 标签在前、双目点云中有人挡在机器狗和标签之间时的低速稳定绕障。它保留 UWB seed 和点云输入方式，但把 APF 斥力替换为 VFH 方向直方图、左右绕障侧锁定和通道清空恢复逻辑。

```text
/libAoa_robot_publisher
  -> uwb_target_seed_node
  -> /stereo_vfh/seed_target

/local_grid_obstacle
  -> stereo_vfh_controller_node
  -> VFH 扇区膨胀 + 左右绕障锁定
  -> /stereo_vfh/follow_goal
  -> /stereo_vfh/markers
  -> /cmd_vel
```

VFH 默认参数更保守，速度上限为 `max_vx: 0.45`、`max_vy: 0.35`、`max_vyaw: 0.8`。首次联调建议先让底盘停止消费 `/cmd_vel`，只观察话题：

```bash
ros2 launch go2_stereo_apf_follow stereo_vfh_follow.launch.py \
  start_rtabmap:=true \
  start_uwb:=true \
  uwb_device:=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AG00S82A-if00-port0
```

检查 VFH 输入、状态和可视化：

```bash
ros2 topic echo /stereo_vfh/seed_target --once
ros2 topic echo /stereo_vfh/target
ros2 topic echo /stereo_vfh/follow_goal
ros2 topic echo /stereo_vfh/status
ros2 topic echo /cmd_vel
```

## 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --packages-select uwb_aoa_pkg go2_stereo_apf_follow --symlink-install
source install/setup.bash
```

## 启动

首次联调先不要让底盘消费速度，只观察 `/cmd_vel`：

```bash
ros2 launch go2_stereo_apf_follow stereo_apf_follow.launch.py \
  start_rtabmap:=true \
  start_uwb:=true \
  uwb_device:=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AG00S82A-if00-port0
```

检查输入和输出：

```bash
ros2 topic hz /local_grid_obstacle
ros2 topic hz /libAoa_robot_publisher
ros2 topic echo /stereo_apf/seed_target --once
ros2 topic echo /stereo_apf/target
ros2 topic echo /stereo_apf/status
ros2 topic echo /cmd_vel
```

如果 UWB 坐标已经是 `base_footprint` 下的前方 x、左方 y，可以启动时加：

```bash
uwb_frame_id:=base_footprint
```

手动目标测试：

```bash
ros2 topic pub --once /stereo_apf/manual_target geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'base_footprint'}, pose: {position: {x: 0.8, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

## 当前保守默认参数

核心默认值：

- `follow_distance: 1.00`
- `apf_emergency_dist: 0.35`
- `apf_slowdown_dist: 0.80`
- `apf_influence_dist: 0.80`
- `apf_repulse_gain: 0.01`
- `max_vx: 0.45`
- `max_vy: 0.25`
- `max_vyaw: 0.8`
- `allow_reverse: false`
- `reverse_scale: 0.8`

点云只做当前帧过滤，不做时间保持：

- `obstacle_x_min: -4.0`
- `obstacle_x_max: 4.0`
- `obstacle_y_abs: 4.0`
- `obstacle_z_min: 0.05`
- `obstacle_z_max: 1.2`
- `robot_frame_front: 0.15`
- `robot_frame_back: 0.35`
- `robot_frame_left: 0.15`
- `robot_frame_right: 0.15`

## 实机提醒

默认速度已经降低，但第一次实测仍建议架空或临时把 `max_vx` 降到 `0.1`。双目点云视野比 2D 激光窄，侧后方障碍不要依赖本包处理。
