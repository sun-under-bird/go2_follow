# Go2 UWB Smac Hybrid + TEB 跟随避障

本包用于 Unitree Go2 的 UWB 目标跟随。节点直接订阅 `/libAoa_robot_publisher`，只根据 `uwb_aoa_pkg/msg/LibAoaRobotMsg.state` 判断目标是否有效，不使用 `pos_confidence`、RSSI、跳变距离或目标速度过滤。

当 UWB 目标距离小于 `follow_distance_m`，默认 1 米时，节点会取消正在进行的规划和跟踪，向 `/uwb_teb/path` 发布空路径，并向 `/cmd_vel_nav` 发布零速度。

## 数据流

```text
/libAoa_robot_publisher
  -> uwb_teb_follow_node
  -> /compute_path_to_pose
  -> planner_server / SmacPlannerHybrid
  -> /uwb_teb/path
  -> /follow_path
  -> controller_server / TEB
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel_safe

/local_grid_obstacle
  -> global_costmap + local_costmap

/local_grid_ground
  -> global_costmap + local_costmap clearing
```

## 主要行为

- `state >= 0`：接收 UWB 目标。
- `state < 0`：判为无效，停车并清空路径。
- 优先使用消息中的 `x/y` 作为目标点；如果 `x/y` 无效或为零，则回退到 `r/a`，其中 `a` 默认按弧度处理。
- UWB 点先转换到 `base_link` 做距离判断，再转换到 `odom` 作为 Smac Hybrid 的规划目标。
- Smac goal 直接使用 UWB 点，不提前计算 1 米外的跟随点。
- 距离小于 `follow_distance_m`、UWB 超时、TF 失败、规划失败或路径太短时，都会发布空路径和零速度。

## Nav2 配置

`config/nav2_smac_teb.yaml` 启动：

- `planner_server`
- `controller_server`
- `global_costmap`
- `local_costmap`
- `velocity_smoother`

全局规划器：

- 插件：`nav2_smac_planner/SmacPlannerHybrid`
- 搜索模型：`DUBIN`
- 在 `odom` rolling `global_costmap` 中规划，不依赖静态地图

局部控制器：

- 默认插件：`teb_local_planner/TEBLocalPlanner`
- 如果实机安装的 TEB 插件导出名不同，只需要修改 `config/nav2_smac_teb.yaml` 中 `FollowPath.plugin` 这一行。

地图：

- 不启动 `map_server`
- 不配置 `static_layer`
- `global_costmap` 和 `local_costmap` 都是 `odom` 下 rolling window
- 两个 costmap 都订阅 `/local_grid_obstacle` 和 `/local_grid_ground`
- 两个 costmap 都在障碍层后加入 `nav2_costmap_2d::DenoiseLayer`，用于过滤孤立点云噪声，再交给膨胀层处理

## 运行

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install --packages-select uwb_aoa_pkg go2_uwb_teb_follow
source install/setup.bash
```

启动 UWB 跟随、Smac Hybrid、TEB 和速度平滑：

```bash
ros2 launch go2_uwb_teb_follow uwb_teb_nav2.launch.py
```

速度桥接节点应订阅：

```text
/cmd_vel_safe
```

## 关键参数

配置文件：`config/uwb_teb_follow.yaml`

- `uwb_topic`：UWB 原始消息，默认 `/libAoa_robot_publisher`
- `follow_distance_m`：跟随停止距离，默认 `1.0`
- `target_timeout_sec`：UWB 超时停车时间，默认 `1.0`
- `planner_action`：默认 `/compute_path_to_pose`
- `planner_id`：默认 `GridBased`
- `follow_path_action`：默认 `/follow_path`
- `controller_id`：默认 `FollowPath`
- `stop_cmd_vel_topic`：停车零速度输出，默认 `/cmd_vel_nav`

## 实机检查

```bash
ros2 node list
ros2 action list | grep compute_path_to_pose
ros2 action list | grep follow_path
ros2 topic echo /follow/uwb_teb_status
ros2 topic echo /follow/target_valid
ros2 topic echo /uwb_teb/target
ros2 topic echo /uwb_teb/path --once
ros2 topic echo /cmd_vel_safe --once
```

典型现象：

- `state >= 0` 且 `pos_confidence = 0` 时，目标仍应被接收。
- `state < 0` 时，应发布空路径并停车。
- UWB 距离 `< 1.0m` 时，`/cmd_vel_nav` 应收到零速度，`/uwb_teb/path` 应为空。
- UWB 距离 `>= 1.0m` 时，应请求 `/compute_path_to_pose`。
- Smac 返回路径后，`/uwb_teb/path` 应出现路径，TEB 通过 `/follow_path` 跟踪。
- costmap 不依赖静态地图，只使用 `/local_grid_obstacle` 和 `/local_grid_ground`，并通过 DenoiseLayer 过滤孤立噪点。

## 限制

这里的“全局路径”是在 `odom` rolling `global_costmap` 中计算，不是静态大地图导航。目标超出 rolling 窗口、被障碍物完全隔开或需要长距离绕行时，Smac 可能规划失败，节点会停车并等待新的 UWB 目标。
