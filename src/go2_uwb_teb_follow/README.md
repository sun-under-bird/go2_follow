# Go2 UWB Smac Hybrid + TEB 跟随避障

该包先把 UWB 人员位置转换为保持 `1.0 m` 距离的站位点，再让 Smac Hybrid 在 rolling global costmap 中规划路径，最后由 TEB 跟踪。与 DWB/直连 MPPI 相比，它能在局部窗口内生成明确的绕障路径，但依赖更多 Nav2 插件。

## 数据流

```text
/libAoa_robot_publisher
  -> uwb_teb_follow_node
  -> ComputePathToPose / SmacPlannerHybrid
  -> /uwb_teb/path
  -> follow_path_recovery_bt_node
       FollowPath 失败 -> BackUpTwzFree -> 重试
  -> controller_server / TEB
  -> /cmd_vel
```

`uwb_teb_follow_node` 只持有规划 action；规划成功后发布路径，由行为树持有 `FollowPath`。节点不检查 UWB 状态、置信度或超时，收到第一帧后持续使用最后目标；进入保持距离或规划失败时发布空路径，行为树会撤销控制/恢复 action。

## 启动

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install --packages-up-to go2_uwb_teb_follow
source install/setup.bash

ros2 launch go2_dynamic_follow_avoidance d435i_rtabmap.launch.py
ros2 launch go2_uwb_teb_follow uwb_teb_nav2.launch.py
```

需要系统已安装 TEB 与 STVL。若 TEB 插件导出名不同，修改 `config/nav2_smac_teb.yaml` 中 `FollowPath.plugin`。

## 检查

```bash
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
ros2 topic echo /uwb_teb/path --once
ros2 topic echo /follow/teb_recovery_status
ros2 topic echo /cmd_vel
```

当前没有 `velocity_smoother` 或额外安全门控，planner、controller 与恢复 behavior 的最终速度出口统一为 `/cmd_vel`。

## 边界

- rolling global costmap 不是持久全局地图，目标超出窗口或窄通道可能规划失败。
- TEB/STVL 不是 ROS 2 Humble 最小 Nav2 安装的必备组件，需要在 Go2 容器核对插件。
- 规划路径更新较慢时，行为树使用最近一次有效路径；UWB 停更不会自动撤销。
