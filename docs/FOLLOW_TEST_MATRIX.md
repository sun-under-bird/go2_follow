# Go2 UWB 跟随方案实机测试矩阵

本文用于把当前所有方案逐一测试。每次只启动一个会发布 `/cmd_vel` 的控制方案；切换前必须 `Ctrl-C` 关闭上一方案并确认进程退出。

## 1. 一次性前置检查

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
colcon build --symlink-install
source install/setup.bash

ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic hz /libAoa_robot_publisher
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/infra2/image_rect_raw
ros2 topic echo /camera/camera/infra1/camera_info --once
ros2 topic echo /camera/camera/infra2/camera_info --once
```

共同约定必须为：里程计 `/odom`、机体 `base_footprint`、最终速度 `/cmd_vel`。检查是否残留旧控制器：

```bash
ros2 topic info /cmd_vel --verbose
ros2 node list | grep -E 'controller|follow|mppi|dwb|teb|robot_nexus'
```

第一轮建议架空 Go2 或保持底盘不进入运动模式，只观察 `/cmd_vel`。接地后先把最大线速度降到 `0.1 m/s`。

## 2. 感知公共链

DWB、MPPI、Dynamic、TEB、APF、VFH、jie 和 EXACT 测试前先提供 D435i 局部点云：

```bash
ros2 launch go2_dynamic_follow_avoidance d435i_rtabmap.launch.py
```

检查：

```bash
ros2 topic hz /local_grid_obstacle
ros2 topic hz /local_grid_ground
ros2 run tf2_ros tf2_echo base_footprint camera_link
```

若方案的 launch 没有 `start_uwb` 参数，另开终端启动外接 UWB：

```bash
ros2 run uwb_aoa_pkg libAoa_robot_example \
  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AG00S82A-if00-port0 \
  --ros-args -p frame_id:=base_footprint -p publish_rate_hz:=10.0
```

## 3. 每个方案的启动命令

| 顺序 | 方案 | 启动命令 | 避障/恢复能力 |
| --- | --- | --- | --- |
| 1 | Dynamic simple | `ros2 launch go2_dynamic_follow_avoidance go2_follow_bringup.launch.py use_simple_follow:=true start_nav2_controller:=false` | 无避障，仅验证 UWB 方向 |
| 2 | `go2_uwb` | `ros2 launch go2_uwb follow_bringup.launch.py` | 最近障碍距离 + 左右转向，无 BT |
| 3 | jie APF+记忆 | `ros2 launch jie_deamon uwb_stereo_follow.launch.py` | APF + 2 秒局部记忆，无 BT |
| 4 | 纯 APF | `ros2 launch go2_stereo_apf_follow stereo_apf_follow.launch.py` | 当前点云势场，无 BT |
| 5 | VFH | `ros2 launch go2_stereo_apf_follow stereo_vfh_follow.launch.py` | 极坐标空闲方向与绕行侧锁定，无 BT |
| 6 | DWB | `ros2 launch go2_uwb_dwb_follow uwb_dwb_nav2.launch.py` | 短轨迹采样 + 空闲方向 BT 恢复 |
| 7 | 直连 MPPI | `ros2 launch go2_uwb_mppi_follow uwb_mppi_nav2.launch.py start_uwb:=false` | 直线路径采样 + 空闲方向 BT 恢复 |
| 8 | Dynamic A*+MPPI | `ros2 launch go2_dynamic_follow_avoidance go2_follow_bringup.launch.py start_rtabmap:=false start_uwb:=false` | 局部 A* + MPPI + 空闲方向 BT 恢复 |
| 9 | Smac+TEB | `ros2 launch go2_uwb_teb_follow uwb_teb_nav2.launch.py` | rolling 规划 + TEB + 空闲方向 BT 恢复 |
| 10 | EXACT-MPPI | `ros2 launch go2_exact_mppi_follow go2_exact_mppi_follow.launch.py` | GPU 采样控制，无 Nav2 BT |

如果 MPPI launch 由自身启动 UWB，可省略 `start_uwb:=false` 和公共 UWB 终端，避免两个串口进程抢占同一设备。

## 4. 每个方案统一测试场景

每个方案都按以下顺序执行并记录 rosbag：

1. 标签静止在正前方 2.5 m：应向前，角速度接近 0。
2. 标签移到左前/右前：角速度方向正确，Go2 不反向转。
3. 标签进入保持距离：速度归零，不持续前冲。
4. 在路径侧边放障碍：控制应绕开或降低朝障碍方向的速度。
5. 在正前方放宽障碍：不能发布穿过 footprint 的速度。
6. 人员横向移动：路径/目标随 UWB 更新，无明显高频 action 抢占。
7. 断开 UWB 3 秒：应撤销路径或归零；恢复数据后重新跟随。
8. 遮挡 D435i：记录该方案的降级行为，当前阶段不要求独立安全门控兜底。

记录建议：

```bash
ros2 bag record /libAoa_robot_publisher /odom /tf /tf_static \
  /local_grid_obstacle /local_grid_ground /follow/target_valid \
  /follow_path /uwb_dwb/path /uwb_follow/path /uwb_teb/path /cmd_vel
```

## 5. DWB 专项测试

DWB 的关键不是“空闲区总分”，而是速度采样轨迹的 critic 总分。建议在 RViz 同时观察 `/local_costmap/costmap`、`/uwb_dwb/path` 和 `/back_up_twz_free_markers`。

```bash
ros2 topic echo /follow/uwb_point_status
ros2 topic echo /follow/dwb_recovery_status
ros2 action list | grep -E 'follow_path|backup'
```

验收现象：

- 侧边障碍：仍处于 `tracking`，DWB 选择不碰撞且总 critic 代价最低的轨迹。
- 正前方封死：先输出零速；约在进度检查/控制失败后状态变为 `recovering`。
- 恢复期间：`/backup` action 激活，绿色 Marker 表示自由区域质心，`/cmd_vel` 应朝该方向。
- 恢复完成：自动回到 `tracking` 并重试最新两点路径。
- 两次恢复失败：状态为 `failed: recovery retries exhausted`，保持停止，直到 UWB 站位目标明显变化。

## 6. 结果记录表

| 方案 | UWB 跟随 | 侧边绕障 | 正前阻挡 | 目标丢失停车 | 恢复成功 | CPU | 结论 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| simple |  | 不适用 | 不适用 |  | 不适用 |  |  |
| go2_uwb |  |  |  |  | 不适用 |  |  |
| jie |  |  |  |  | 不适用 |  |  |
| APF |  |  |  |  | 不适用 |  |  |
| VFH |  |  |  |  | 不适用 |  |  |
| DWB |  |  |  |  |  |  |  |
| MPPI |  |  |  |  |  |  |  |
| Dynamic |  |  |  |  |  |  |  |
| TEB |  |  |  |  |  |  |  |
| EXACT |  |  |  |  | 不适用 |  |  |

每次测试结束都确认：

```bash
ros2 topic info /cmd_vel --verbose
ps -ef | grep -E 'controller_server|behavior_server|follow_path_recovery' | grep -v grep
```

若仍有测试节点，先正常 `Ctrl-C` 关闭对应 launch，不要带着旧 controller 启动下一方案。
