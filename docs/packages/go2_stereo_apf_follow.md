# `go2_stereo_apf_follow` 使用说明

## 包定位

该 C++ 包提供两条互斥控制器：

- APF：当前帧点云上的人工势场、目标走廊横向修正和急停。
- VFH：极坐标方向直方图、障碍膨胀、左右绕行锁定和通道清空恢复。

两条路线共用 `uwb_target_seed_node`，并允许手动目标测试。它们不使用 Nav2，也没有独立 safety mux，直接发布 `/cmd_vel`。

## 共同 UWB seed 节点

`uwb_target_seed_node` 解析 `LibAoaRobotMsg` 的 x/y 或 r/a，按配置翻转 y、补角度/锚点偏移，通过 TF 转到 `base_frame`，直接发布原始 seed 目标；不检查状态、置信度、距离或超时。

| 方向 | APF 默认话题 | VFH 默认话题 | 类型 |
| --- | --- | --- | --- |
| 订阅 | `/libAoa_robot_publisher` | 同左 | `LibAoaRobotMsg` |
| 发布 | `/stereo_apf/seed_target` | `/stereo_vfh/seed_target` | `PoseStamped` |

## APF 控制器

### 输入输出

| 方向 | 默认话题 | 类型 |
| --- | --- | --- |
| 订阅 | `/local_grid_obstacle` | `PointCloud2` |
| 订阅 | `/stereo_apf/seed_target` | `PoseStamped` |
| 订阅 | `/stereo_apf/manual_target` | `PoseStamped` |
| 订阅 | `/stereo_apf/enabled` | `Bool` |
| 发布 | `/cmd_vel` | `Twist` |
| 发布 | `/stereo_apf/target` | `PoseStamped` |
| 发布 | `/stereo_apf/status` | `String` |
| 发布 | `/stereo_apf/potential_field` | `MarkerArray`，可选 |

### 算法

1. 点云转到 `base_footprint`，过滤 ROI、高度和机器人自身矩形。
2. 直接使用最新 UWB seed 作为目标，不做点云质心跟踪或低通。
3. 障碍小于影响距离时计算二维斥力。
4. 目标走廊左右占用生成 `linear.y`。
5. 最近障碍小于急停距离直接零速，减速区间缩放 `vx`。

默认关键值：跟随 `1.0 m`、障碍影响/减速 `0.8 m`、急停 `0.35 m`、最大 `vx/vy/wz=0.45/0.25/0.8`，禁止后退。

## VFH 控制器

### 输入输出

VFH 使用 `/stereo_vfh/*` 命名空间，并额外发布：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/stereo_vfh/follow_goal` | `PoseStamped` | 扣除跟随距离后的站位点 |
| `/stereo_vfh/markers` | `MarkerArray` | 直方图扇区、目标方向和选择方向 |

### 算法与状态机

- 将障碍按 `sector_angle_deg=5°` 写入 360° 直方图。
- 按 `robot_radius + safety_margin` 对每个点做角度膨胀。
- 屏蔽 UWB 目标 `target_mask_radius` 内的人体点。
- 目标走廊或目标扇区被阻塞时，在左右自由扇区中评分。
- 首次选择后锁定 LEFT/RIGHT；只有锁定侧无路且保持时间结束才切换。
- 走廊连续清空 `corridor_clear_hold_sec` 后恢复 FOLLOW。
- VFH 计算结果不经过低通或各轴变化率限制，直接发布。

默认关键值：硬停 `0.35 m`、减速 `0.80 m`、机器人半径 `0.35 m`、安全余量 `0.20 m`、最大 `vx/vy/wz=0.45/0.35/0.8`。

## launch 文件

`stereo_apf_follow.launch.py` 与 `stereo_vfh_follow.launch.py` 可选启动本工作区 RTAB-Map 和 UWB 驱动。RTAB-Map 固定消费 D435i 已矫正红外双目图与对应 CameraInfo。

主要参数：`config_file`、`base_frame`、`pointcloud_topic`、`cmd_vel_topic`、各组件 `start_*`、UWB 设备/frame/频率和 `use_sim_time`。

最终速度默认直接发布到 `/cmd_vel`，没有额外转发话题。

## 编译与启动

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install \
  --packages-select uwb_aoa_pkg go2_stereo_apf_follow
source install/setup.bash
```

已有 UWB 与 `/local_grid_obstacle` 时：

```bash
ros2 launch go2_stereo_apf_follow stereo_vfh_follow.launch.py \
  start_rtabmap:=false \
  start_uwb:=false
```

APF：

```bash
ros2 launch go2_stereo_apf_follow stereo_apf_follow.launch.py
```

手动目标：

```bash
ros2 topic pub --once /stereo_vfh/manual_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_footprint'}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}"
```

## 检查

```bash
ros2 topic hz /local_grid_obstacle
ros2 topic echo /stereo_vfh/status
ros2 topic echo /stereo_vfh/follow_goal --once
ros2 topic echo /cmd_vel
```

## 已知边界

- APF 有局部极小值和障碍移除后立即恢复的问题；VFH 仍不是空间路径规划。
- 两个控制器直接输出底盘速度，没有输入超时 watchdog；目标或点云停更后会持续使用最后一帧。
- 点云订阅使用默认 reliable QoS；接 best-effort 点云源时需验证兼容性。
- 每帧最多遍历 60000 点，且没有内建体素降采样。
- 大量 TF 查询使用最新时间，机器人运动时可能引入时序误差。
- 前视点云无法保证侧后方安全，尤其是 VFH 的 360° 直方图并不等于 360° 感知。
