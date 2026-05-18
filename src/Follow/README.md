# Go2 Jetson 动态跟随避障

这个 ROS2 包实现一条局部动态跟随链路：

```text
ONE1000 /libAoa_robot_publisher
        -> follow_goal_node
        -> /follow_goal
        -> local_path_planner
        -> /follow_path
        -> Nav2 FollowPath / MPPI
        -> /cmd_vel
        -> safety_mux
        -> /cmd_vel_safe
        -> 你已有的 Go2 速度桥接
```

包内不直接调用 Unitree Go2 SDK。你已经有速度桥接时，只需要让桥接订阅 `/cmd_vel_safe`。

## 功能

- 从 ONE1000 的定位消息中读取 `x/y` 或 `r/angle`，生成保持距离后的动态跟随目标。
- 从已标定双目的 `/stereo/points2` 生成前向 rolling costmap。
- 用 A* 在局部 costmap 上生成短 `nav_msgs/Path`，不依赖全局地图。
- 通过 `FollowPath` action 持续把短路径送给 Nav2 Controller Server / MPPI。
- 对 `/cmd_vel` 做最后安全门控，异常时输出零速度到 `/cmd_vel_safe`。

## 依赖

Jetson 上建议使用 Ubuntu 22.04 + ROS2 Humble：

```bash
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-nav2-mppi-controller \
  ros-humble-stereo-image-proc \
  ros-humble-sensor-msgs-py
```

你的双目需要先输出标准 ROS2 图像和 `camera_info`，再用 `stereo_image_proc` 生成点云：

```bash
ros2 run stereo_image_proc stereo_image_proc --ros-args \
  -r left/image_rect:=/stereo/left/image_rect \
  -r right/image_rect:=/stereo/right/image_rect \
  -r left/camera_info:=/stereo/left/camera_info \
  -r right/camera_info:=/stereo/right/camera_info \
  -r points2:=/stereo/points2
```

## 编译

把本目录放到 ROS2 工作空间的 `src` 下：

```bash
mkdir -p ~/go2_follow_ws/src
cp -r go2_dynamic_follow_avoidance ~/go2_follow_ws/src/
cd ~/go2_follow_ws
colcon build --symlink-install
source install/setup.bash
```

如果 ONE1000 的消息类型不是默认的 `uwb_aoa_pkg/msg/LibAoaRobot`，启动时改 `one1000_msg_type`，并在配置里改字段名。

## 运行

先启动 TF、双目点云、ONE1000 ROS2 节点、Nav2 Controller Server。Nav2 Controller Server 可以参考：

```bash
ros2 run nav2_controller controller_server --ros-args \
  --params-file ~/go2_follow_ws/src/go2_dynamic_follow_avoidance/config/nav2_mppi_controller.yaml
```

再启动本包：

```bash
ros2 launch go2_dynamic_follow_avoidance dynamic_follow_avoidance.launch.py \
  one1000_topic:=/libAoa_robot_publisher \
  one1000_msg_type:=uwb_aoa_pkg/msg/LibAoaRobot \
  pointcloud_topic:=/stereo/points2 \
  cmd_vel_in:=/cmd_vel \
  cmd_vel_out:=/cmd_vel_safe
```

最后让你的 Go2 速度桥接订阅 `/cmd_vel_safe`。

## 主要话题

- `/one1000/target`：ONE1000 目标点，位于 `base_link`。
- `/follow_goal`：保持 1.5m 跟随距离后的机器人局部目标。
- `/local_costmap`：本包内部 A* 使用的调试 costmap。
- `/follow_path`：送给 Nav2 MPPI 的动态短路径。
- `/follow/path_valid`：短路径是否有效。
- `/cmd_vel_safe`：安全门控后的速度，接你的 Go2 速度桥接。
- `/follow/safety_status`：安全状态文本，便于调试。

## 关键参数

默认参数在 `config/go2_dynamic_follow_avoidance.yaml`：

- `follow_distance: 1.5`
- `confidence_threshold: 50.0`
- `target_timeout_sec: 0.3`
- `pointcloud_timeout_sec: 0.5`
- `inflation_radius: 0.35`
- `emergency_x_max: 0.45`
- `max_vx: 0.2`
- `max_vy: 0.3`
- `max_vyaw: 0.8`

调通后可以把 `safety_mux.max_vx` 从 `0.2` 提到 `0.5`。

## 坐标约定

- `base_link`：Go2 机体坐标，前方为 `+x`，左方为 `+y`。
- `one1000_anchor`：ONE1000 锚点坐标。默认通过 TF 转到 `base_link`。
- `odom`：Nav2 Controller Server 使用的局部路径坐标系。

必须保证以下 TF 可用：

```text
odom -> base_link
base_link -> stereo_link -> stereo_left_optical_frame
base_link -> one1000_anchor
```

如果 ONE1000 节点已经直接输出 `base_link` 下的 `x/y`，可以把 `follow_goal_node.use_tf` 设为 `false`。

## 调试顺序

1. RViz 检查 `/stereo/points2` 是否在 `base_link` 前方。
2. RViz 检查 `/one1000/target` 是否指向信令实际位置。
3. 观察 `/local_costmap` 中障碍是否正确膨胀。
4. 观察 `/follow_path` 是否能绕开障碍生成短路径。
5. 架空或限速测试 `/cmd_vel_safe`。
6. 最后接入 Go2 速度桥接低速实测。

## 安全边界

本方案只使用前向双目，侧后方是盲区。默认禁止倒车，前方 0.45m 内有障碍时强制急停。第一次实测请保持 `max_vx: 0.2`，并确保遥控器能随时接管。
