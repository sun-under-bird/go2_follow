# Go2 UWB 点目标 DWB 跟随避障

本包用于在 ROS 2 Humble 下测试 Unitree Go2 的 UWB 点目标跟随。它不做全局规划，也不做 A* 或历史轨迹路径规划，只根据当前 UWB 点计算一个保持距离后的跟随目标，再交给 Nav2 DWB 做局部速度采样和避障。

注意：Nav2 Humble 的 DWB 控制器接口仍然是 `FollowPath`，所以节点会生成一个只有两个点的“接口适配 path”：

```text
当前机器人位置 -> UWB 跟随目标点
```

这个 path 不是规划路径，只是为了调用 DWB。DWB 的参数也按“目标点趋近 + 障碍避让”配置，弱化了路径贴合相关 critic。

## 运行链路

```text
UWB 当前点
  -> uwb_point_follow_node
  -> /uwb_follow_target
  -> 两点式 FollowPath goal
  -> Nav2 FollowPath(DWB)
  -> /cmd_vel_safe

双目或深度点云
  -> /local_grid_obstacle
  -> Nav2 local costmap
  -> DWB BaseObstacle critic
```

本包只启动 Nav2 `controller_server`，不启动全局 planner、BT navigator 或 recoveries。

## 主要文件

- `src/uwb_point_follow_node.cpp`：订阅 UWB 消息，计算跟随目标，并刷新 DWB `FollowPath` goal。
- `config/uwb_dwb_follow.yaml`：UWB 点目标、跟随距离、滤波和 action 刷新参数。
- `config/nav2_dwb_controller.yaml`：DWB 控制器和局部代价地图参数。
- `launch/uwb_dwb_follow.launch.py`：只启动 UWB 点目标跟随节点。
- `launch/uwb_dwb_nav2.launch.py`：启动 DWB controller、生命周期管理器和 UWB 点目标跟随节点。

## 依赖

需要安装 Nav2 DWB 相关包：

```bash
sudo apt install ros-humble-dwb-core ros-humble-dwb-plugins ros-humble-dwb-critics
```

本包还依赖 `uwb_aoa_pkg` 提供 UWB 消息类型。

## 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install --packages-select uwb_aoa_pkg go2_uwb_dwb_follow
source install/setup.bash
```

## 启动前检查

先确认 Go2 里程计、TF、UWB 和点云正常：

```bash
ros2 topic hz /odom_leg
ros2 topic hz /libAoa_robot_publisher
ros2 topic hz /local_grid_obstacle
ros2 topic hz /local_grid_ground
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link uwb_link
```

## 启动

启动完整 DWB 点目标跟随链路：

```bash
ros2 launch go2_uwb_dwb_follow uwb_dwb_nav2.launch.py
```

第一次实测建议先不要直接接 Go2 执行端，把输出改到观察话题：

```bash
ros2 launch go2_uwb_dwb_follow uwb_dwb_nav2.launch.py cmd_vel_out:=/cmd_vel_nav
```

确认速度方向和幅值正常后，再输出到 `/cmd_vel_safe` 或接入你的安全门控。

## 调试话题

```bash
ros2 topic echo /follow/uwb_point_status
ros2 topic echo /follow/target_valid
ros2 topic echo /uwb_follow_target
ros2 topic echo /follow_path
ros2 topic echo /cmd_vel_safe
```

RViz2 中建议观察：

- `/uwb_follow_target`
- `/follow_path`
- `/local_costmap/costmap`
- `/local_grid_obstacle`

其中 `/follow_path` 只是两点式接口适配 path，不代表系统做了路径规划。

## DWB 初始参数

默认按低速差速模型设置：

- 最大前进速度：`0.45 m/s`
- 最大后退速度：`0.0 m/s`
- 最大角速度：`1.0 rad/s`
- 轨迹预测时间：`1.5 s`
- 局部代价地图：`4 m x 4 m`
- 膨胀半径：`0.30 m`

如果 UWB 点抖动，可以优先调 `smoothing_alpha`、`max_target_jump_m` 和 `max_target_speed_mps`。如果遇到完全挡住直达方向的大障碍，DWB 可能会停住或局部试探；这是点目标直跟方案的限制，因为它没有 A* 或全局路径来主动绕远路。
