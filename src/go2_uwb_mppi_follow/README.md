# Go2 UWB MPPI 跟随避障

本功能包用于实现“方案 B”：将 UWB 发布的目标相对坐标转换到 `odom` 坐标系，维护一段短时目标历史路径，再生成机器狗滞后跟随的参考路径，交给 Nav2 MPPI 控制器结合双目局部地图完成避障跟随。

## 核心思路

```text
UWB 相对目标坐标
  -> TF 转换到 odom
  -> 跳点、速度、距离过滤
  -> 短时目标历史路径
  -> 按跟随距离生成 /follow_path
  -> Nav2 FollowPath(MPPI)
  -> cmd_vel

前置双目 / 深度点云
  -> Nav2 local costmap
  -> MPPI 障碍物代价
```

UWB 用来回答“目标在哪里、刚才怎么走”，双目局部地图用来回答“机器狗前方哪里不能走”。本包不直接处理点云，也不直接输出速度，而是输出 MPPI 可以跟踪的局部参考路径。

## 主要节点

### `uwb_path_tracker_node`

订阅 UWB 目标数据，生成短时历史路径和 MPPI 参考路径。

输入：

- `/libAoa_robot_publisher`：UWB 数据，类型为 `uwb_aoa_pkg/msg/LibAoaRobotMsg`
- TF：`odom -> base_link -> uwb_link`
- Nav2 FollowPath action：默认 action 名称为 `follow_path`

输出：

- `/uwb_target_history`：已接收的 UWB 目标历史路径，类型为 `nav_msgs/msg/Path`
- `/follow_path`：发送给 MPPI 跟踪的滞后参考路径，类型为 `nav_msgs/msg/Path`
- `/follow/target_valid`：目标是否有效，类型为 `std_msgs/msg/Bool`
- `/follow/uwb_path_status`：节点状态，类型为 `std_msgs/msg/String`

## 必要 TF

至少需要：

```text
odom -> base_link
base_link -> uwb_link
```

如果 UWB 坐标已经是 `base_link` 下的目标坐标，可以在参数中设置：

```yaml
use_tf_for_uwb: false
```

此时节点会把 UWB 坐标当成 `base_link` 坐标使用。

## 关键参数

配置文件位于：

```text
config/uwb_mppi_follow.yaml
```

常用参数：

- `follow_distance_m`：机器狗与目标保持的距离，默认 `1.2`
- `history_length_m`：保留的 UWB 历史路径长度，默认 `6.0`
- `min_sample_spacing_m`：相邻 UWB 历史点最小间距，默认 `0.10`
- `max_target_jump_m`：允许的最大 UWB 跳变距离，默认 `1.5`
- `max_target_speed_mps`：允许的目标最大速度，默认 `3.0`
- `target_timeout_sec`：目标超时时间，默认 `2.0`
- `controller_id`：Nav2 MPPI 控制器 ID，默认 `FollowPath`

## 运行方式

编译：

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install --packages-select go2_uwb_mppi_follow
source install/setup.bash
```

启动本包节点：

```bash
ros2 launch go2_uwb_mppi_follow uwb_mppi_follow.launch.py
```

如果要指定参数文件：

```bash
ros2 launch go2_uwb_mppi_follow uwb_mppi_follow.launch.py \
  params_file:=/path/to/uwb_mppi_follow.yaml
```

## 与 Nav2 MPPI 的关系

本包只负责生成 `/follow_path` 并发送 `FollowPath` action。MPPI 控制器、local costmap、双目点云输入仍然需要由 Nav2 配置提供。

典型 Nav2 local costmap 输入可以是：

```text
/local_grid_obstacle
/local_grid_ground
```

MPPI 会根据 `/follow_path` 的参考路径和 local costmap 的障碍物代价，选择一条短时安全轨迹并输出速度。

## 调试建议

先不要直接接入机器狗速度执行，建议先观察：

```bash
ros2 topic echo /follow/uwb_path_status
ros2 topic echo /follow/target_valid
ros2 topic echo /follow_path
ros2 topic echo /uwb_target_history
```

在 RViz2 中查看：

- `/uwb_target_history`
- `/follow_path`
- Nav2 local costmap

确认路径方向、距离和 TF 坐标都正确后，再接入 MPPI 输出速度和机器狗执行端。

## 当前限制

- 依赖短时 `odom` 和 yaw 稳定性。
- UWB 多径或跳点严重时，需要继续收紧过滤参数。
- 只有前置双目时，应保持低速，并在 local costmap 无效时停车。
- 本包不会替代 Nav2 local costmap，也不会直接处理深度图或点云。
