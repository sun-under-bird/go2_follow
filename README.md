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
矫正双目图像 -> stereo_image_proc/BM -> 深度观测（障碍点 + 自由空间射线）
                                      -> rolling_obstacle_map_node + /leg_odom2 pose
                                      -> local_velocity_planner_node -> /cmd_vel
```

局部规划器使用自研 MPPI 优化时变速度序列，并读取 `/leg_odom2` 的线速度和角速度
作为轨迹预测初值；独立的滚动障碍地图
节点使用同一里程计的位置和朝向补偿历史障碍。局部规划和滚动地图均为二维，不需要
全局地图或全局路径。

## 外部输入

启动本仓库前，需要机器人系统提供：

```text
/camera/camera/infra1/camera_info
/camera/camera/infra1/image_rect_raw
/camera/camera/infra2/camera_info
/camera/camera/infra2/image_rect_raw
/leg_odom2
base_footprint -> camera optical frame 的 TF
```

图像必须已经完成双目校正。仓库使用 `stereo_image_proc` 的 BM 视差算法，不使用 SGBM。

## 编译

```bash
cd /home/cat/robot_ws/go2_follow_develop
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

确认 UWB、双目图像、TF 和 `/leg_odom2` 正常后，启动完整跟随避障链路：

```bash
ros2 launch go2_uwb_local_follow local_follow.launch.py \
  enable_motion:=true \
  cmd_vel_topic:=/cmd_vel \
  odom_topic:=/leg_odom2
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
- MPPI 默认每周期以 `48` 条控制序列执行 `2` 轮加权更新，并保留完整制动安全校验。
- 普通规划保持非负线速度；负速度只用于经过后向扫掠检查的紧急恢复状态机。
- 点云和里程计同时校验采集时间与接收时间；障碍按里程计补偿到当前机身坐标。
- 点云采集年龄超过 `0.30 s` 后逐渐降速，`0.70 s` 停车，新鲜输入恢复后自动重规划。
- UWB 串口遇到非法长度、CRC 错误或超过 `200 ms` 的半帧会重新寻帧，拔插后每秒重连。
- 每周期只建立一次障碍空间索引，所有 MPPI 候选共用并保留精确足迹碰撞检查。
- 紧急区使用新点云连续帧确认，单帧近场伪点不会直接锁存紧急停车。
- 滚动局部地图使用深度射线清除已观测自由空间，并以时间衰减清理未被再次观测的障碍。

详细参数和调节说明位于：

```text
src/go2_uwb_local_follow/config/uwb_follow_only.yaml
src/go2_uwb_local_follow/config/stereo_obstacle_cloud.yaml
src/go2_uwb_local_follow/config/rolling_obstacle_map.yaml
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

需要把实机异常和完整数据移交给测试或研发人员时，使用一键记录脚本：

```bash
./scripts/record_follow_test.sh --name rear_crossing
```

脚本支持按键标记异常向前、转向、误停车、反光假障碍、抖动和 UWB 丢数；详细用法见
[跟随避障实机测试记录说明](docs/follow_avoidance_test_recording.md)，可直接移交测试人员的
命令清单见 [Go2 跟随避障测试录制命令](docs/follow_test_recording_commands.md)。

## 目录

```text
src/
├── go2_uwb_local_follow/  # 跟随、双目障碍点云和局部速度规划
└── uwb/                   # UWB 串口驱动与消息定义
```
