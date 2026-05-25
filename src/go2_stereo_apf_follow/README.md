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
  -> 目标邻域质心更新
  -> APF 急停/减速/排斥
  -> /cmd_vel
```

参考仓库没有独立安全门控。它的急停在 APF 控制内部完成：最近障碍距离 `< 0.20m` 时发布 0 速度；障碍离开后下一帧自动恢复。

## 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --packages-select uwb_aoa_pkg go2_stereo_apf_follow --symlink-install
source install/setup.bash
```

## 启动

首次联调先不要接速度桥，只观察 `/cmd_vel`：

```bash
ros2 launch go2_stereo_apf_follow stereo_apf_follow.launch.py \
  start_camera:=true \
  start_rtabmap:=true \
  start_uwb:=true \
  start_twist_bridge:=false \
  uwb_device:=/dev/ttyUSB0
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

如果 UWB 坐标已经是 `base_link` 下的前方 x、左方 y，可以启动时加：

```bash
uwb_frame_id:=base_link
```

手动目标测试：

```bash
ros2 topic pub --once /stereo_apf/manual_target geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'base_link'}, pose: {position: {x: 0.8, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

## 与参考仓库一致的参数

核心默认值：

- `follow_distance: 0.40`
- `apf_emergency_dist: 0.20`
- `apf_slowdown_dist: 0.25`
- `apf_influence_dist: 0.25`
- `apf_repulse_gain: 0.01`
- `max_vx: 1.0`
- `max_vy: 1.0`
- `max_vyaw: 1.0`
- `allow_reverse: true`
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

现在默认速度上限和参考仓库一样是 `1.0`，第一次实测仍建议架空或临时把 `max_vx` 降低。双目点云视野比 2D 激光窄，侧后方障碍不要依赖本包处理。
