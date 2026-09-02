# Go2 UWB 双目跟随避障 v1.0.0

本仓库只保留已经用于实机验证的 UWB 跟随与双目局部避障链路，目标平台为
Unitree Go2、Ubuntu 22.04 和 ROS 2 Humble。

## 保留的 ROS 2 包

| 包 | 作用 |
| --- | --- |
| `uwb_aoa_pkg` | 读取 Ubitraq UWB/AoA 串口，发布 `/libAoa_robot_publisher` |
| `go2_uwb_local_follow` | UWB 目标适配、距离跟随、双目 BM 深度、障碍点云、局部速度规划与安全停车 |

数据链路：

```text
UWB 串口
  -> /libAoa_robot_publisher
  -> uwb_target_adapter_node
  -> /uwb/target_point
  -> uwb_follow_controller_node
  -> /go2_uwb_local_follow/nominal_cmd
                                      \
矫正双目图像 -> stereo_image_proc/BM -> 障碍点云 -> local_velocity_planner_node
                                                      -> /cmd_vel
```

局部规划器只读取 `/odom_leg` 的线速度和角速度作为轨迹预测初值，不使用里程计位姿。

## 外部输入

启动本仓库前，需要机器人系统提供：

```text
/camera/camera/infra1/camera_info
/camera/camera/infra1/image_rect_raw
/camera/camera/infra2/camera_info
/camera/camera/infra2/image_rect_raw
/odom_leg
base_footprint -> camera optical frame 的 TF
```

图像必须已经完成双目校正。仓库使用 `stereo_image_proc` 的 BM 视差算法，不使用 SGBM。

## 编译

```bash
cd /root/go2_follow_worktree
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to go2_uwb_local_follow
source install/setup.bash
```

`src/uwb/lib/uwb_robot_algo.a` 是 ARM64 厂商静态库。在 ARM64 上默认构建串口驱动；
其他架构只生成 UWB 消息接口，便于上层算法使用 rosbag 或模拟消息测试。

## 启动

先启动 UWB 串口驱动：

```bash
ros2 launch uwb_aoa_pkg uwb_source.launch.py \
  serial_port:=/dev/ttyUSB0
```

确认 UWB、双目图像、TF 和 `/odom_leg` 正常后，启动完整跟随避障链路：

```bash
ros2 launch go2_uwb_local_follow local_follow.launch.py \
  enable_motion:=true \
  cmd_vel_topic:=/cmd_vel \
  odom_topic:=/odom_leg
```

第一次调试建议使用隔离输出：

```bash
ros2 launch go2_uwb_local_follow local_follow.launch.py \
  enable_motion:=false
```

## v1.0.0 关键控制参数

- 期望跟随距离：`1.0 m`，距离死区：`0.08 m`。
- 最大跟随线速度：`0.8 m/s`。
- 最大跟随角速度：`2.0 rad/s`。
- 主动避障角速度范围：`0.5~1.5 rad/s`。
- 最大角加速度：`2.0 rad/s²`。
- 避障优先保持 UWB 名义线速度，只有当前速度层不存在安全转向轨迹时才分级降速。
- 紧急区使用新点云连续帧确认，单帧近场伪点不会直接锁存紧急停车。

详细参数和调节说明位于：

```text
src/go2_uwb_local_follow/config/uwb_follow_only.yaml
src/go2_uwb_local_follow/config/stereo_obstacle_cloud.yaml
src/go2_uwb_local_follow/config/local_velocity_planner.yaml
```

## 诊断与测试

主要诊断话题：

```text
/uwb/target_adapter_diagnostics
/go2_uwb_local_follow/follow_diagnostics
/stereo/obstacle_diagnostics
/go2_uwb_local_follow/planner_diagnostics
```

单元测试：

```bash
colcon test --packages-select go2_uwb_local_follow
colcon test-result --verbose
```

实机运行前应确认只有一个节点发布 `/cmd_vel`，并在机器人周围预留安全空间。

## 目录

```text
src/
├── go2_uwb_local_follow/  # 跟随、双目障碍点云和局部速度规划
└── uwb/                   # UWB 串口驱动与消息定义
```
