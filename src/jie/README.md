# jie_deamon

`jie_deamon` 是一个精简的 ROS 2 跟随避障节点。当前版本只保留核心能力：

- UWB 提供跟随目标位置。
- 前向双目点云转换成 `/scan` 后提供前方障碍信息。
- 节点根据目标和障碍计算速度并发布 `/cmd_vel`。

本项目不再包含 Android App、Web 可视化、键盘控制、直控模式和动作指令。

## ROS 2 接口

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/uwb_target` | `geometry_msgs/msg/PointStamped` | UWB 目标位置 |
| 订阅 | `/scan` | `sensor_msgs/msg/LaserScan` | 双目点云转换后的前向扫描 |
| 发布 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 底盘速度指令 |

UWB 目标坐标必须已经在机器人本体坐标系下：

- `point.x`：机器人前方为正。
- `point.y`：机器人左侧为正。
- `point.z`：当前节点忽略。

节点不做 TF 坐标转换。

## 控制策略

跟随目标距离默认为 `1.0m`。机器人只允许前进和转向：

- 不自动后退，`linear.x >= 0`。
- 不横移，`linear.y = 0`。
- 目标角度偏差过大时原地转向，不前进。
- UWB 目标超时后发布零速度。

前向避障默认参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `follow_dist` | `1.0` | 跟随目标距离 |
| `apf_influence_dist` | `0.3` | 斥力影响距离 |
| `apf_slowdown_dist` | `0.3` | 前方减速距离 |
| `apf_emergency_dist` | `0.2` | 急停距离 |
| `target_exclusion_radius` | `0.35` | 排除 UWB 目标自身点云的半径 |
| `target_timeout_sec` | `0.5` | UWB 目标超时时间 |
| `scan_timeout_sec` | `0.5` | `/scan` 超时时间 |
| `max_linear_speed` | `0.5` | 最大前进速度 |
| `max_angular_speed` | `1.0` | 最大角速度 |

`/scan` 中落在 UWB 目标附近 `target_exclusion_radius` 范围内的点不会参与避障排斥，避免跟随目标本身触发斥力。

## 启动

默认 launch 会同时启动点云转激光节点和核心跟随节点：

```bash
ros2 launch jie_deamon uwb_stereo_follow.launch.py
```

默认双目点云输入为 `/stereo/points`，转换后的扫描输出为 `/scan`：

```bash
ros2 launch jie_deamon uwb_stereo_follow.launch.py cloud_in:=/camera/depth/points
```

默认点云转换参数：

| 参数 | 默认值 |
| --- | --- |
| `target_frame` | `base_link` |
| `angle_min` | `-0.6` |
| `angle_max` | `0.6` |
| `range_min` | `0.2` |
| `range_max` | `3.0` |
| `min_height` | `0.05` |
| `max_height` | `0.8` |
| `use_inf` | `true` |

## 注意事项

- 前向双目只能覆盖前方视场，本节点不承诺侧后方避障。
- 如果 UWB 只提供距离而没有方向，不能直接用于本节点；需要上游先解算为机器人坐标系下的二维位置。
- 斥力距离 `0.3m` 和急停距离 `0.2m` 比较接近，实际机器人速度较高时建议在设备上重新评估安全余量。
- 本机没有 ROS 2 环境，代码修改后不做编译检查。
