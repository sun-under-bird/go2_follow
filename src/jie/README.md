# jie_deamon

`jie_deamon` 是一个精简后的 ROS 2 跟随避障节点。当前版本只保留核心能力：

- UWB 提供跟随目标位置。
- 前向双目点云转换成 `/scan` 后提供前方障碍信息。
- 节点根据 UWB 目标、前方可通行 gap 和急停/减速逻辑发布 `/cmd_vel_safe`。

本项目不再包含 Android App、Web 可视化、键盘控制、直控模式和动作指令。

## ROS 2 接口

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/libAoa_robot_publisher` | `uwb_aoa_pkg/msg/LibAoaRobotMsg` | UWB 目标位置 |
| 订阅 | `/scan` | `sensor_msgs/msg/LaserScan` | 双目点云转换后的前向扫描 |
| 发布 | `/cmd_vel_safe` | `geometry_msgs/msg/Twist` | 底盘安全速度指令 |

UWB 目标坐标默认来自 `uwb_link`，节点会通过 TF 转换到 `base_footprint` 后再控制：

- `x`：`uwb_link` 下的前方为正。
- `y`：`uwb_link` 下的左侧为正。
- `state`：必须等于 `1` 才认为定位有效。
- `pos_confidence`：当前不参与过滤，不管置信度多少都跟随。

运行时需要 TF 树中存在 `base_footprint` 到 `uwb_link` 的变换链。如果你已经发布 `base_link` 到 `uwb_link`，还需要系统里同时有 `base_footprint` 到 `base_link` 的变换。`uwb_input_frame` 只在 UWB 消息没有填写 `header.frame_id` 时作为兜底值。

## 控制策略

跟随目标距离默认是 `1.0m`。机器人只使用前进和转向绕障：

- 不自动后退：`linear.x >= 0`。
- 不横移绕障：`linear.y = 0`。
- 前方没有近距离障碍时，按 UWB 目标方向跟随。
- 前方出现近距离障碍时，节点把障碍按 `gap_min_width` 膨胀到角度栅格中，寻找连续可通行 gap。
- gap 选择优先接近 UWB 目标方向，同时惩罚过大的转向和上一帧差异，避免左右抖动。
- 选中 gap 后，机器人用 `angular.z` 平滑转向 gap，再用 `linear.x` 前进。
- 小于 `apf_slowdown_dist` 时开始减速，小于 `apf_emergency_dist` 时急停。
- UWB 目标超时或 `/scan` 超时后发布零速度。

`/scan` 中落在 UWB 目标附近 `target_exclusion_radius` 范围内的点会先被排除，不参与急停、减速和 gap 占用，避免跟随目标自身点云触发避障。

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `follow_dist` | `1.0` | 跟随目标距离 |
| `target_timeout_sec` | `0.5` | UWB 目标超时时间 |
| `scan_timeout_sec` | `0.5` | `/scan` 超时时间 |
| `target_exclusion_radius` | `0.35` | 排除 UWB 目标自身点云的半径 |
| `apf_influence_dist` | `0.8` | 前方减速斥力影响距离，主要用于降低 `linear.x` |
| `apf_slowdown_dist` | `0.8` | 前方减速距离 |
| `apf_emergency_dist` | `0.4` | 急停距离 |
| `max_linear_speed` | `0.5` | 最大前进速度 |
| `max_angular_speed` | `1.0` | 最大角速度 |
| `gap_detection_dist` | `1.5` | 在该距离内的障碍会参与 gap 占用计算 |
| `gap_min_width` | `0.45` | 可通行 gap 的最小安全宽度，用于障碍角度膨胀 |
| `gap_turn_penalty` | `0.35` | 惩罚过大的转向角，越大越偏向走正前方 |
| `gap_stability_penalty` | `0.8` | 惩罚与上一帧 gap 差异，越大越不容易左右跳变 |
| `gap_heading_smoothing_alpha` | `0.35` | gap 方向平滑系数，越小越平滑但响应越慢 |
| `uwb_input_frame` | `uwb_link` | UWB 消息缺省坐标系 |
| `target_frame` | `base_footprint` | 跟随控制和 `/scan` 投影使用的目标坐标系 |

## 启动

默认 launch 会同时启动点云转激光节点和核心跟随节点：

```bash
ros2 launch jie_deamon uwb_stereo_follow.launch.py
```

默认障碍点云输入为 `/local_grid_obstacle`，转换后的扫描输出为 `/scan`：

```bash
ros2 launch jie_deamon uwb_stereo_follow.launch.py cloud_in:=/camera/depth/points
```

如果 `/local_grid_obstacle` 不是 `sensor_msgs/msg/PointCloud2`，请把 `cloud_in` 改成实际的 PointCloud2 话题，例如 `/local_costmap/voxel_marked_cloud`。当前节点不订阅 `/odom_leg`，里程计只需要由系统用于维护 `base_footprint`、`base_link`、`uwb_link`、点云坐标系之间的 TF。

默认点云转换参数：

| 参数 | 默认值 |
| --- | --- |
| `target_frame` | `base_footprint` |
| `angle_min` | `-1.57` |
| `angle_max` | `1.57` |
| `angle_increment` | `0.0087` |
| `range_min` | `0.01` |
| `range_max` | `3.0` |
| `min_height` | `0.02` |
| `max_height` | `0.8` |
| `use_inf` | `true` |

当前点云转换会把 `base_footprint` 坐标系下高度约 `2cm` 到 `0.8m`、水平角约 `-90` 到 `90` 度的点压到 XY 平面生成 `/scan`。如果输入是原始双目点云，地面点或噪声可能影响避障，更推荐输入已经去地面后的障碍点云，例如 `/local_grid_obstacle`。

## 注意事项

- 前向双目只能覆盖前方视场，本节点不承诺侧后方避障。
- gap 选择是局部避障，不是全局路径规划；如果当前视野里没有足够宽的 gap，节点会停止前进。
- 如果 UWB 只提供距离而没有方向，不能直接用于本节点；上游需要先解算为二维位置。
- 当前 launch 默认在 `1.5m` 内识别 gap，占用 `0.8m` 内开始减速，`0.4m` 内急停；实际机器人速度较高时仍建议在设备上重新评估安全余量。
- 本机没有 ROS 2 环境，代码修改后不做编译检查。
