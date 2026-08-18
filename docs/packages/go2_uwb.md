# `go2_uwb` 使用说明

## 包定位

`go2_uwb` 是一条独立的 C++ 轻量路线：转换原始 UWB 目标、用 PCL 从点云估算最近障碍和绕行方向、再由比例控制器直接输出速度。它不使用 Nav2、路径规划或 costmap。

## 数据流

```text
/libAoa_robot_publisher
  -> uwb_filter_node
  -> /uwb/target_point
                         ┐
/local_grid_obstacle     │
  -> obstacle_detector_node
  -> /obstacle/nearest_distance
  -> /obstacle/avoid_vector
                         ├-> follow_controller_node -> /cmd_vel
                         ┘
```

`follow_bringup.launch.py` 不再启动任何相机或图像处理节点，直接消费 RTAB-Map 输出的 `/local_grid_obstacle`。

## 节点说明

### `uwb_filter_node`

| 方向 | 默认话题 | 类型 |
| --- | --- | --- |
| 订阅 | `/libAoa_robot_publisher` | `LibAoaRobotMsg` |
| 发布 | `/uwb/target_point` | `PointStamped` |

参数包括输入/输出话题、`output_frame`、回退输入 frame 和 TF 超时。节点不检查状态或置信度，把原始目标严格变换到 `base_footprint`；TF 失败时拒绝发布，禁止只改 frame 标签。

### `obstacle_detector_node`

订阅 `/local_grid_obstacle`，通过最新 TF 转到 `target_frame=base_footprint`，然后：

1. 过滤非有限点和前向 ROI。
2. 可选 VoxelGrid 体素降采样。
3. 可选 RadiusOutlierRemoval。
4. 统计最近 xy 距离和左右点数。
5. 点数小于阈值时发布 `inf` 代表无障碍。
6. 左右点数差超过死区时选择点更少的一侧，否则保持上次方向。

| 方向 | 默认话题 | 类型 |
| --- | --- | --- |
| 订阅 | `/local_grid_obstacle` | `PointCloud2`，SensorDataQoS |
| 发布 | `/obstacle/nearest_distance` | `Float32` |
| 发布 | `/obstacle/avoid_vector` | `Vector3Stamped`，使用 `vector.z` 表示建议角速度 |
| 发布 | `/obstacle/used_points` | `PointCloud2`，可选调试输出 |

launch 默认 ROI：`x 0.2~2.0 m`、`|y|<=0.8 m`、`z 0.08~1.0 m`；体素 `0.05 m`；至少 8 个点才视为障碍。

### `follow_controller_node`

订阅目标、最近障碍距离和绕行角速度建议，输出速度与状态。

核心行为：

- 直接使用最近一次原始 UWB 点，不做跳变、低通或目标超时停车。
- 根据目标距离误差计算前进速度，根据方位计算角速度。
- 障碍小于 `avoid_distance` 进入绕行锁存，超过 `avoid_release_distance` 释放。
- 障碍小于 `front_stop_distance` 时停止前进。
- 最终速度只做最大值裁剪，不做加速度或低通平滑，直接发布 `/cmd_vel`。
- 还未收到障碍数据时也允许按目标跟随；收到后持续使用最近障碍结果。

| 方向 | 默认话题 | 类型 |
| --- | --- | --- |
| 订阅 | `/uwb/target_point` | `PointStamped` |
| 订阅 | `/obstacle/nearest_distance` | `Float32` |
| 订阅 | `/obstacle/avoid_vector` | `Vector3Stamped` |
| 发布 | `/cmd_vel` | `Twist` |
| 发布 | `/go2_uwb/controller_status` | `String` |

launch 默认：目标距离 `1.5 m`、最大前进 `0.5 m/s`、最大角速度 `0.8 rad/s`、前停 `0.45 m`、避障进入/释放 `0.9/1.05 m`。

## 编译与启动

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install --packages-select uwb_aoa_pkg go2_uwb
source install/setup.bash

ros2 launch go2_uwb follow_bringup.launch.py
```

launch 提供 `pointcloud_topic` 和 `cmd_vel_topic` 入口；控制阈值仍可分别启动节点并传参数覆盖。

启动前必须已有：

- `/libAoa_robot_publisher`。
- 需提前用 D435i 已矫正红外双目输入生成 `/local_grid_obstacle`。
- 相机 frame 到 `base_footprint` 的 TF。

## 检查

```bash
ros2 topic hz /uwb/target_point
ros2 topic hz /local_grid_obstacle
ros2 topic echo /obstacle/nearest_distance
ros2 topic echo /obstacle/avoid_vector
ros2 topic echo /go2_uwb/controller_status
ros2 topic echo /cmd_vel --once
```

## 已知边界

- UWB 或障碍数据停更后不会自动停车，会继续复用最后数据。
- 左右点数只能给出简单转向建议，不能保证绕过凹形或封闭障碍。
- 绕障只修改角速度，没有显式目标人体点云清除。
- 只有前向 ROI，没有侧后方传感器覆盖或独立 safety mux。
