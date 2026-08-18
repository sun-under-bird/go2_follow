# Go2 UWB DWB 跟随避障

该包用于验证“当前 UWB 点 → 保持距离站位点 → 两点参考路径 → DWB 局部避障”。它不运行全局规划器；路径只包含机器人当前位置和站位点，用于适配 Nav2 `FollowPath` 接口。

## 运行链

```text
/libAoa_robot_publisher
  -> uwb_point_follow_node
  -> /uwb_dwb/path
  -> follow_path_recovery_bt_node
       FollowPath 失败 -> BackUp -> 重试 FollowPath
  -> controller_server / DWB
  -> /cmd_vel

/local_grid_obstacle + /local_grid_ground
  -> local_costmap
  -> DWB critics + BackUpTwzFree 空闲方向搜索
```

行为树 XML 位于 `behavior_ext_plugins/behavior_trees/follow_path_with_free_space_recovery.xml`。`BackUp` 仍是 Nav2 标准行为树节点，但 `behavior_server.backup.plugin` 已替换为 `nav2_behaviors/BackUpTwzFree`：DWB 找不到有效速度或进度检查失败后，行为树会先朝局部代价地图中的自由区域移动 `0.30 m`，再重试原路径，默认最多恢复两次。

## DWB 如何选速度

DWB 不是简单计算一张“空闲区域得分图”后直接发速度。它会：

1. 在当前速度和加速度约束内采样 `(vx, vy, wz)`。
2. 对每组速度向前模拟 `1.5 s` 的短轨迹。
3. 用 `BaseObstacle` 排除碰撞轨迹。
4. 用 `GoalAlign`、`GoalDist`、`RotateToGoal`、`Oscillation` 等 critic 对剩余轨迹评分。
5. 选择总分最低的轨迹，并把该轨迹的首个速度直接发布到 `/cmd_vel`。

当前 `max_vel_y=0`，所以 DWB 控制阶段按差速模型运行；恢复插件可利用 Go2 的横移能力发布 `linear.y`。

## 编译与启动

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install --packages-up-to go2_uwb_dwb_follow
source install/setup.bash

ros2 launch go2_uwb_dwb_follow uwb_dwb_nav2.launch.py
```

该 launch 启动 `controller_server`、`behavior_server`、生命周期管理器、UWB 路径节点和行为树执行器。它不启动 UWB 串口驱动、D435i 点云链或里程计。

## 启动前检查

```bash
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic hz /libAoa_robot_publisher
ros2 topic hz /local_grid_obstacle
ros2 topic hz /local_grid_ground
```

## 运行检查

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
ros2 action list | grep -E 'follow_path|backup'
ros2 topic echo /follow/uwb_point_status
ros2 topic echo /follow/dwb_recovery_status
ros2 topic echo /uwb_dwb/path --once
ros2 topic info /cmd_vel --verbose
ros2 topic echo /cmd_vel
```

RViz 建议显示 `/uwb_follow_target`、`/uwb_dwb/path`、`/local_costmap/costmap`、`/local_grid_obstacle` 和 `/back_up_twz_free_markers`。

## 当前边界

- DWB 只在短预测窗内绕障，无法像 A* 或 Smac 那样规划长距离绕路。
- 完全封死目标方向时会先停车，待 `FollowPath` 返回失败后才进入恢复，不会穿越碰撞轨迹。
- 恢复两次仍失败时行为树停止并等待站位目标明显变化；不会无限反复脱困。
- 当前按需求没有速度平滑器和独立安全门控；一次只能启动一个 `/cmd_vel` 控制方案。
