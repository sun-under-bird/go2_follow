# Go2 Dynamic UWB 跟随避障

该包直接使用原始 UWB 目标，结合 D435i 局部障碍栅格生成局部 A* 路径，再由 Nav2 MPPI 跟踪。与直连 MPPI 不同，它先在局部栅格中生成绕障路径。

```text
UWB -> follow_goal_node -> /one1000/target
D435i 点云 -> local_path_planner -> /follow_path
/follow_path -> follow_path_recovery_bt_node
             -> MPPI FollowPath -> /cmd_vel
             -> 失败时 BackUpTwzFree -> 重试
```

当前阶段按要求移除了 `safety_mux`、速度平滑器和旧的 Python `follow_path_action_client`。controller、行为树恢复以及 simple 模式都直接使用 `/cmd_vel`；一次只能启动一个方案。

目标链不再检查 UWB 状态、置信度、连续样本、跳变、速度或输入超时；收到第一帧后持续复用最后一个原始目标。

## 标准启动

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install --packages-up-to go2_dynamic_follow_avoidance
source install/setup.bash

ros2 launch go2_dynamic_follow_avoidance go2_follow_bringup.launch.py \
  start_rtabmap:=true \
  start_uwb:=true
```

D435i 输入固定为已矫正话题：

- `/camera/camera/infra1/image_rect_raw`
- `/camera/camera/infra2/image_rect_raw`
- `/camera/camera/infra1/camera_info`
- `/camera/camera/infra2/camera_info`

不启动 `image_proc`、`stereo_image_proc`、图像拼接或拆分节点。

## Simple 模式

Simple 模式仅用于不接相机时核对 UWB 方向，不提供障碍规避或恢复：

```bash
ros2 launch go2_dynamic_follow_avoidance go2_follow_bringup.launch.py \
  use_simple_follow:=true \
  start_nav2_controller:=false
```

## 检查

```bash
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /follow/path_valid
ros2 topic echo /follow/dynamic_recovery_status
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
ros2 topic info /cmd_vel --verbose
```

主要限制是局部栅格窗口和 Python A* 性能；大范围、U 形或完全封闭障碍不保证可解。
