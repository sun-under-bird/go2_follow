# `jie_deamon` 使用说明

## 包定位

源码目录是 `src/jie`，ROS 包名和 CMake project 是 `jie_deamon`。它把前向点云转换为二维 LaserScan，维护一个 `odom` 下的短时局部障碍记忆，并用 APF + 目标走廊占用直接输出 `/cmd_vel`。

## 数据流

```text
/local_grid_obstacle
  -> pointcloud_to_laserscan
  -> /scan
  -> LocalObstacleMap（可选）
                         ┐
/libAoa_robot_publisher  │
  -> TF 到 base_footprint     ├-> FollowAvoidController -> /cmd_vel
                         ┘
```

## 节点与内部类

### `robot_nexus`

| 方向 | 默认话题 | 类型 |
| --- | --- | --- |
| 订阅 | `/scan` | `sensor_msgs/msg/LaserScan`，SensorDataQoS |
| 订阅 | `/libAoa_robot_publisher` | `LibAoaRobotMsg` |
| 发布 | `/cmd_vel` | `geometry_msgs/msg/Twist` |

UWB 处理不检查 `state` 或置信度，使用消息 header frame；为空时回退到 `uwb_input_frame=base_footprint`。收到第一帧目标和 scan 后直接复用最新数据，没有输入超时 watchdog。

### `LocalObstacleMap`

- scan 点先转到 `local_map_frame=odom`。
- 每帧可沿可见射线清除旧点。
- 同一 `local_map_resolution` 邻域内的点合并并平滑。
- 只保留机器人周围 `local_map_size_x × local_map_size_y` 窗口。
- 超过 `local_map_lifetime_sec` 的点删除。
- 点数限制为 `local_map_max_points`。
- 控制前把地图点转回 `target_frame`。

TF 失败时回退到仅使用当前 scan，不会因为局部地图失效而完全停机。

### `FollowAvoidController`

控制逻辑：

- 排除机器人 footprint 内和 UWB 目标半径内的 scan 点。
- 累加 `apf_influence_dist` 内的斥力，并限制斥力幅值。
- 统计机器人到目标矩形走廊的左右占用，生成横向速度。
- 最近障碍小于 `apf_emergency_dist` 时零速。
- 小于 `apf_slowdown_dist` 时缩放前进速度。
- 方位角大于 `rotate_only_angle` 时只转向，不前进。
- 不主动后退，最终 `linear.x >= 0`。

## launch 参数

`uwb_stereo_follow.launch.py` 暴露：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `cloud_in` | `/local_grid_obstacle` | 输入点云 |
| `scan_topic` | `/scan` | 转换后的扫描 |
| `target_frame` | `base_footprint` | 控制坐标系 |
| `local_map_frame` | `odom` | 障碍记忆固定坐标系 |
| `local_map_enabled` | `true` | 是否启用短时地图 |
| `uwb_target_topic` | `/libAoa_robot_publisher` | UWB 输入 |
| `cmd_vel_topic` | `/cmd_vel` | 最终速度输出 |
| `uwb_input_frame` | `base_footprint` | UWB header 为空时的 frame |

PointCloud→LaserScan 固定为前方 `[-1.57, 1.57] rad`、0.5° 分辨率、`0.01~3.0 m`、高度 `0.02~0.8 m`。

## 关键控制参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `follow_dist` | `1.0` | 跟随距离 |
| `target_exclusion_radius` | `0.35` | 目标人体点排除半径 |
| `apf_influence_dist` | `0.6` | 斥力范围 |
| `apf_emergency_dist` | `0.3` | 急停距离 |
| `max_linear_speed` | `0.5` | 最大前进速度 |
| `max_lateral_speed` | `0.12` | 最大横移速度 |
| `max_angular_speed` | `1.0` | 最大角速度 |
| `rectangle_width` | `0.4` | 目标走廊宽度 |
| `local_map_size_x/y` | `1.0 / 1.0` | 短时地图大小 |
| `local_map_resolution` | `0.05` | 点合并距离 |
| `local_map_lifetime_sec` | `2.0` | 障碍记忆寿命 |

全部参数见 `src/jie/README.md` 和 `robot_nexus.cpp::loadConfig()`。

## 编译与启动

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install --packages-select uwb_aoa_pkg jie_deamon
source install/setup.bash

ros2 launch jie_deamon uwb_stereo_follow.launch.py \
  cloud_in:=/local_grid_obstacle
```

示例使用测试话题以便台架观察；正式启动默认直接输出 `/cmd_vel`。

没有 `odom` TF 时可关闭短时地图：

```bash
ros2 launch jie_deamon uwb_stereo_follow.launch.py \
  local_map_enabled:=false
```

## 检查

```bash
ros2 topic hz /scan
ros2 topic echo /cmd_vel
ros2 run tf2_ros tf2_echo odom base_footprint
```

## 已知边界

- 局部地图只能记住曾进入前向视场的障碍，不是全向感知。
- 当前地图点插入会线性扫描已有 vector，密集点云下复杂度较高。
- `linear.y` 只有在实际 Go2 底盘接口支持横移时才有效。
- 无速度/加速度平滑或独立 mux，急停外的命令可能随 scan 帧变化。
- UWB 和 scan 停更时不会自动停车，而会继续使用最后一帧数据；这是按当前测试阶段要求移除安全超时后的直接后果。
- launch 中算法参数大多固定，只有少数通用参数暴露为 launch argument。
