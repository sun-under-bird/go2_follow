# Go2 双目 APF 跟随避障

这个包是在现有工作区中新增的 C++ ROS2 包，用双目相机/RTAB-Map 的障碍点云实现类似 `jie_deamon` 的低速跟随避障方法。它不依赖原仓库源码，也不启动 Nav2、A* 或 MPPI。

## 方法概览

```text
/libAoa_robot_publisher
  -> uwb_target_seed_node
  -> /stereo_apf/seed_target
  -> /stereo_apf/seed_valid

/local_grid_obstacle
  -> stereo_apf_controller_node
  -> 点云过滤 + 目标邻域质心跟踪 + APF 排斥力
  -> /stereo_apf/target
  -> /cmd_vel_apf

apf_safety_mux_node
  -> 超时停车 + 近障碍急停 + 限速限加速度
  -> /cmd_vel_safe
  -> go2_twist_bridge
  -> Go2
```

目标可以来自 UWB，也可以通过 `/stereo_apf/manual_target` 手动发布 `PoseStamped`。UWB 只提供目标种子；控制器会在当前目标附近查找双目点云质心，持续更新目标位置。找不到质心时会短暂保持，随后回退到新鲜 UWB 种子，仍不可用则停车。

## 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --packages-select uwb_aoa_pkg go2_stereo_apf_follow --symlink-install
source install/setup.bash
```

如果 `uwb_aoa_pkg` 已经编译过，可以只编译本包：

```bash
colcon build --packages-select go2_stereo_apf_follow --symlink-install
```

## 启动顺序

先确认 Go2、TF、相机和 RTAB-Map 能正常工作：

```bash
ros2 topic hz /local_grid_obstacle
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo base_link uwb_link
```

第一次联调不要直接接速度桥，只观察速度：

```bash
ros2 launch go2_stereo_apf_follow stereo_apf_follow.launch.py \
  start_uwb:=true \
  start_camera:=false \
  start_rtabmap:=false \
  start_twist_bridge:=false \
  uwb_device:=/dev/ttyUSB0
```

观察：

```bash
ros2 topic echo /stereo_apf/status
ros2 topic echo /stereo_apf/safety_status
ros2 topic echo /stereo_apf/target
ros2 topic echo /cmd_vel_apf
ros2 topic echo /cmd_vel_safe
```

确认方向、速度、急停都正确后，再启动桥接：

```bash
ros2 launch go2_stereo_apf_follow stereo_apf_follow.launch.py \
  start_seed:=false \
  start_controller:=false \
  start_safety:=false \
  start_twist_bridge:=true
```

## 一键启动

联调稳定后可以让 launch 同时启动 UWB、相机、RTAB-Map 和速度桥：

```bash
ros2 launch go2_stereo_apf_follow stereo_apf_follow.launch.py \
  start_go2_driver:=true \
  start_uwb:=true \
  start_camera:=true \
  start_rtabmap:=true \
  start_twist_bridge:=true \
  uwb_device:=/dev/ttyUSB0
```

## 手动目标

如果暂时没有 UWB，可以手动发布一个 `base_link` 下的目标：

```bash
ros2 topic pub --once /stereo_apf/manual_target geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'base_link'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

也可以临时关闭或开启 APF 控制：

```bash
ros2 topic pub --once /stereo_apf/enabled std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /stereo_apf/enabled std_msgs/msg/Bool "{data: true}"
```

## 关键参数

参数文件在：

```text
config/stereo_apf_follow.yaml
```

常用调参项：

- `follow_distance`：默认 `2.0`，机器人希望保持的前向距离。
- `target_radius`：默认 `0.30`，目标附近点云质心搜索半径。
- `apf_influence_dist`：默认 `1.2`，障碍排斥力影响距离。
- `apf_emergency_dist`：默认 `0.45`，障碍进入该距离直接停车。
- `apf_slowdown_dist`：默认 `1.0`，障碍进入该距离开始减速。
- `max_vx`：默认 `0.30`，首次实测建议保持低速。
- `robot_frame_*`：用于排除机器人自身结构被点云扫到的区域。

## 安全边界

首次实测必须低速或架空，遥控器随时准备接管。确认 `/cmd_vel_safe` 的方向、限速、急停和目标丢失停车都正确以后，再启动 `go2_twist_bridge`。双目相机视野比 2D 雷达窄，侧后方障碍不要依赖本包处理。
