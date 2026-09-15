# Go2 UWB 双目跟随避障 v1.0.0

本仓库提供 UWB 跟随、双目局部避障与行为控制链路，目标平台为
Unitree Go2、Ubuntu 22.04 和 ROS 2 Humble。

## 保留的 ROS 2 包

| 包 | 作用 |
| --- | --- |
| `uwb_aoa_pkg` | 读取 Ubitraq UWB/AoA 串口，发布 `/libAoa_robot_publisher` |
| `go2_uwb_local_follow` | UWB 目标适配、距离跟随、双目 BM 深度、滚动地图、局部速度规划与安全停车 |
| `go2_uwb_behavior` | FOLLOW／IDLE／STOP 与随机漫游 Action，含短暂断流后的任务恢复 |

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
                                      -> rolling_obstacle_map_node + /odom_leg pose
                                      -> local_velocity_planner_node -> /cmd_vel
```

局部规划器读取 `/odom_leg` 的线速度和角速度作为轨迹预测初值；独立的滚动障碍地图
节点使用同一里程计的位置和朝向补偿历史障碍。局部规划和滚动地图均为二维，不需要
全局地图或全局路径。

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
cd /home/bird/go2_follow_rolling_map
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to go2_uwb_behavior
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

## 当前完整链路的关键控制参数

- 期望跟随距离：`1.0 m`，距离死区：`0.08 m`。
- 最大跟随线速度：`0.8 m/s`。
- 完整链路最大角速度：`1.5 rad/s`；纯 UWB 控制器的名义上限为 `2.0 rad/s`。
- 主动避障角速度范围：`0.5~1.5 rad/s`。
- 规划器线加速度／减速度：`0.5 / 0.7 m/s²`；角加速度：`1.5 rad/s²`。
- 预测时域：`1.5 s`，积分步长：`0.05 s`，足迹额外安全余量：`0.05 m`。
- 避障优先保持 UWB 名义线速度，只有当前速度层不存在安全转向轨迹时才分级降速。
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
ROS_DOMAIN_ID=196 ROS_LOCALHOST_ONLY=1 colcon test \
  --packages-select uwb_aoa_pkg go2_uwb_local_follow go2_uwb_behavior \
  --return-code-on-test-failure
colcon test-result --verbose
```

实机运行前应确认只有一个节点发布 `/cmd_vel`，并在机器人周围预留安全空间。

## 目录

```text
src/
├── go2_uwb_behavior/      # 行为控制、漫游 Action 与断流恢复
├── go2_uwb_local_follow/  # 跟随、双目障碍点云和局部速度规划
└── uwb/                   # UWB 串口驱动与消息定义
```

## 流畅跟随与断流恢复

构建默认使用 `RelWithDebInfo`，显式指定的 `Debug` 等构建类型仍有效。碰撞检查使用
每周期共享的空间索引；保留精确旋转矩形净空、速度分层与完整制动尾段。实际下发
速度已经包含在第一控制周期的碰撞预测中，停车目标直接发零速，不会被上一条高速
命令重新抬升。绕障方向偏好由 `avoidance_direction_hold_sec` 独立控制，当前配置为 `0.0`。

所有带 Header 的关键输入必须使用同一个 ROS 时间基准；多机实机系统需要同步
相机、里程计和 UWB 主机时钟，rosbag 回放应统一设置 `use_sim_time`。不再把旧消息
重新盖上当前时间。重复、乱序、零时间戳和明显未来时间戳不会刷新输入有效期。

- 障碍点在每个控制周期补偿到当前机身位姿。采集年龄超过 `0.30 s` 开始降低速度
  上限，超过 `0.70 s` 停车；无法配对里程计时也停车等待。收到新鲜数据后自动恢复。
- UWB 超过 `0.50 s`、规划器里程计超过 `0.10 s` 或名义速度超过 `0.20 s` 停车；
  FOLLOW 始终保留跟随模式，新鲜输入恢复后重新规划，无需重启节点。
- 串口设备启动时不存在或运行中拔出，驱动保持运行并约每秒重新尝试打开。同一
  设备路径恢复后重新发布；建议使用稳定的 `/dev/serial/by-id/...` 路径。半帧超过
  `200 ms`、异常长度和错误 CRC 都会自动丢弃，后续合法帧可以重新同步。
- 急停倒退最多使用 `0.40 m` 命令距离，速度 `0.30 m/s`；后向障碍保留范围自动
  覆盖足迹、完整制动距离和一周期余量。旧点云不能启动或继续倒退。后方障碍清除后
  可重新确认停稳并重试，断流重试不会重置累计倒退预算。
- `go2_uwb_behavior` 的漫游输入超时先进入 `ROAM_INPUT_PAUSED` 并输出零速；在
  `input_recovery_timeout_sec=3.0` 内恢复全部输入且确认停稳后，继续原目标。持续
  断流则返回 `INPUT_TIMEOUT`；Action 总超时仍然有效。STOP 和取消始终优先。

带安全门控的行为链路启动方式见 [行为包说明](src/go2_uwb_behavior/README.md)。
两种完整启动方式选择其一，避免多个控制节点竞争同一名义速度或底盘话题。

### 大角度跟随与绕障协同

普通跟随在目标偏角达到 `heading_stop_angle=1.05 rad`（约 `60.2°`）时停止前进并对准。
新增 `heading_alignment_hysteresis=0.15 rad`，进入对准后要回落到约 `51.6°` 才恢复普通前进，
减少角度门限附近的反复停走。

完整跟随避障链路及行为链路的 FOLLOW 默认启用 `enable_avoidance_heading_relaxation`。
规划器只有在完整轨迹通过检查、当前正在前进绕障且没有进入急停恢复时，才在
`/go2_uwb_local_follow/avoidance_feedback` 发布非零 `TwistStamped` 反馈。
跟随器检查该反馈的源时间戳、坐标系与接收年龄，有效期为 `0.20 s`。

反馈有效且目标仍在跟随距离之外时，大角度绕障允许临时使用
`avoidance_heading_stop_angle=1.48 rad`（约 `84.8°`），名义前进速度最多
`avoidance_heading_max_linear_speed=0.50 m/s`。每周期仍由规划器验证实际指令与完整制动轨迹。
绕障结束、受阻、反馈过期或目标接近时撤销许可；超过扩展角度同样停止前进。
反馈用于延续已经建立的安全绕障，从一开始就大角度停车的状态不会无条件获得前进许可。
一旦进入对准，延后收到的绕障反馈也不能提前解除停车滞回。

纯 UWB 跟随默认关闭角度放宽；漫游继续使用自身目标策略。角度协同不依赖绕障方向保持参数。
跟随诊断增加 `avoidance_heading_relaxed` 和 `heading_alignment`；行为 FOLLOW 放宽时状态为
`FOLLOW_AVOIDANCE_HEADING_RELAXED`。

两种完整启动文件均可通过 `enable_avoidance_heading_relaxation:=false` 关闭角度放宽。

离线性能基准（固定随机种子，200／2000／5000 个障碍点，不启动 ROS 节点）：

```bash
./build/go2_uwb_local_follow/benchmark_local_planner
```

程序输出规划中位耗时、P95 和最大耗时。实机还应结合诊断里的输入年龄、
`planning_time_ms` 和底盘实际制动能力验证控制周期；合成基准不替代实机避障验收。

2026-09-15 本地 x86_64 验证：同种子、同参数、5000 个侧墙障碍点、21 个候选，
原未优化构建中位耗时 `125.59 ms`，原算法启用 `-O2` 后 `68.98 ms`，
共享空间索引后 `0.56 ms`。以上为各 15 次测量；新算法使用当前配置的 `0.05 s`
积分步长、首周期约束和 `0.05 m` 安全余量时，预热后 100 次测量中位耗时
`0.58 ms`、P95 `1.48 ms`。数值仅代表本机合成场景，Go2 板载计算机需另行测量。
