# jie_deamon

`jie_deamon` 是一个 ROS 2 UWB 跟随避障节点。当前版本使用 UWB 提供跟随目标位置，使用前向相机点云转换出的 `/scan` 提供前方障碍信息，并维护一个短时局部地图来补足侧向障碍记忆，然后发布 `/cmd_vel`。

## ROS 2 接口

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/libAoa_robot_publisher` | `uwb_aoa_pkg/msg/LibAoaRobotMsg` | UWB 目标位置 |
| 订阅 | `/scan` | `sensor_msgs/msg/LaserScan` | 相机点云转换后的前向扫描 |
| 发布 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 最终底盘速度指令 |

UWB 目标坐标默认来自 `base_footprint`，节点会在需要时通过 TF 转换到 `target_frame` 后再控制：

- `x`：前方为正。
- `y`：左侧为正。
- `state`、`pos_confidence`：均不参与目标门控，控制器直接使用原始坐标。

## 局部地图

节点维护一个以机器人为中心的 `1m x 1m` 局部障碍记忆：

1. `/scan` 中的前向障碍点先通过 TF 写入 `local_map_frame`，默认是 `odom`。
2. 每帧 `/scan` 会先沿相机视野光束清除自由空间内的历史障碍点。
3. 每帧控制前再把地图点转换回 `target_frame`，得到当前机器人周围的障碍点。
4. 地图只保留当前机器人周围 `local_map_size_x x local_map_size_y` 范围内的点。
5. 超过 `local_map_lifetime_sec` 没刷新的点会自动删除。
6. 如果局部地图 TF 不可用，节点会退回到只使用当前 `/scan` 的旧逻辑。

这个局部地图只能记住“之前进入过前向相机视野”的障碍。侧面突然出现、从未被相机看见过的障碍仍然无法感知。

## 控制策略

- 跟随目标距离默认是 `1.0m`。
- 机器人默认不主动后退，`linear.x` 最终会被限制为 `>= 0`。
- 目标角度较大时优先转向，角度较小时才前进。
- 障碍小于 `apf_influence_dist` 时产生 APF 斥力。
- 障碍小于 `apf_slowdown_dist` 时开始减速。
- 障碍小于 `apf_emergency_dist` 时直接急停。
- `/scan` 中落在 UWB 目标附近 `target_exclusion_radius` 范围内的点会被排除，避免跟随对象自身触发避障。
- 目标方向矩形走廊会统计左右占用，并输出 `linear.y` 做侧向避障；实际是否横移取决于底盘是否执行 `Twist.linear.y`。

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `follow_dist` | `1.0` | 跟随目标距离 |
| `target_exclusion_radius` | `0.35` | 排除 UWB 目标自身点云的半径 |
| `apf_influence_dist` | `0.6` | APF 斥力影响距离 |
| `apf_slowdown_dist` | `0.6` | 前方减速距离 |
| `apf_emergency_dist` | `0.3` | 急停距离 |
| `max_linear_speed` | `0.5` | 最大前进速度 |
| `max_lateral_speed` | `0.12` | 最大横向速度 |
| `max_angular_speed` | `1.0` | 最大角速度 |
| `linear_y_scale_factor` | `1.0` | 侧向避障比例系数 |
| `rectangle_width` | `0.4` | 目标方向矩形走廊宽度 |
| `robot_frame_front` | `0.25` | 机器人自身前向排除距离 |
| `robot_frame_back` | `0.25` | 机器人自身后向排除距离 |
| `robot_frame_left` | `0.16` | 机器人自身左侧排除距离 |
| `robot_frame_right` | `0.16` | 机器人自身右侧排除距离 |
| `local_map_enabled` | `true` | 是否启用局部地图 |
| `local_map_frame` | `odom` | 局部地图固定坐标系 |
| `local_map_size_x` | `1.0` | 局部地图前后尺寸，单位 m |
| `local_map_size_y` | `1.0` | 局部地图左右尺寸，单位 m |
| `local_map_resolution` | `0.05` | 障碍点合并分辨率，单位 m |
| `local_map_lifetime_sec` | `2.0` | 障碍点保留时间 |
| `local_map_max_points` | `1600` | 局部地图最大点数 |
| `local_map_ray_clear_enabled` | `true` | 是否开启相机视野射线清除 |
| `local_map_ray_clear_radius` | `0.06` | 射线附近多宽范围内的历史点会被清除 |
| `local_map_ray_clear_hit_margin` | `0.08` | 有命中障碍时，命中点前保留的末端保护距离 |
| `uwb_input_frame` | `base_footprint` | UWB 消息缺省坐标系 |
| `target_frame` | `base_footprint` | 跟随控制和 `/scan` 使用的目标坐标系 |

## 启动

默认 launch 会同时启动点云转激光节点和核心跟随节点：

```bash
ros2 launch jie_deamon uwb_stereo_follow.launch.py
```

默认障碍点云输入为 `/local_grid_obstacle`，转换后的扫描输出为 `/scan`：

```bash
ros2 launch jie_deamon uwb_stereo_follow.launch.py cloud_in:=/camera/depth/points
```

如果没有 `odom -> base_footprint` 或等价 TF，局部地图不能做运动补偿。此时可以临时关闭局部地图：

```bash
ros2 launch jie_deamon uwb_stereo_follow.launch.py local_map_enabled:=false
```

## 注意事项

- 前向相机只能覆盖前方视场，局部地图只提供短期历史记忆，不是全向传感器。
- 局部地图依赖 TF/里程计质量；位姿漂移会让历史障碍点位置不准。
- 地面点或点云噪声会影响避障，建议输入已经去地面后的障碍点云。
- 收到第一帧 UWB 和 scan 后会持续复用最后一帧数据，不再因输入超时自动停车。
