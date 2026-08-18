# Go2 UWB 双目跟随避障工作区

这是一个面向 Unitree Go2、ROS 2 Humble、UWB 人员定位和前向双目感知的实验工作区。仓库中保留了多条可替换的跟随控制路线，用于比较自写 A*、APF、VFH、Nav2 DWB、Nav2 MPPI、Smac Hybrid + TEB 和 JAX EXACT-MPPI。

> 安全提示：这些路线会直接或间接发布机器人速度。按当前测试要求已移除输入超时停车、目标质量门控和速度平滑，传感器停更时可能继续沿用旧数据。首次运行必须架空机器人或让底盘停止消费 `/cmd_vel`，只观察速度话题。

## 先读结论

- 这是“多方案实验仓库”，不是需要全部同时启动的一套系统。
- 一次只能选择一个最终速度控制链路。所有控制器和恢复行为都直接发布 `/cmd_vel`，不再保留中间速度话题或转发节点；多个控制器同时发布仍会产生不可预测的抢占。
- 当前实机配置把外接 UWB 模块与机体前向对齐，完整 launch 将厂家驱动消息标记为 `base_footprint`；若以后改变安装角度，应修改厂家算法输入配置或建立真实 UWB TF，不能只改 frame 名。
- 相机输入统一使用 D435i 已矫正红外双目图和驱动发布的 CameraInfo，不再包含拼接拆图、V4L2、`image_proc` 或 `stereo_image_proc` 链路。
- 当前最完整的局部路线是 `go2_dynamic_follow_avoidance` 的自写 A* + Nav2 MPPI + 行为树恢复，最轻量的路线是 `go2_stereo_apf_follow` 的 APF/VFH，标准控制器对照优先测试 DWB。
- 已完成的结构和安全优化、仍可继续改进的项目见 [优化建议](docs/OPTIMIZATION.md)。

## 包总览

| ROS 包 | 作用 | 主要输出 | 定位 |
| --- | --- | --- | --- |
| `uwb_aoa_pkg` | 串口读取外接 UWB/AoA，直接换算平面坐标并发布自定义消息 | `/libAoa_robot_publisher` | 基础驱动 |
| `go2_dynamic_follow_avoidance` | 原始 UWB 目标、自写局部栅格/A*、Nav2 MPPI 跟踪、行为树恢复 | `/cmd_vel` | 综合方案 |
| `go2_uwb` | PCL 点云过滤、简单左右绕行建议和 UWB 跟随控制 | `/cmd_vel` | 早期 C++ 方案 |
| `jie_deamon` | PointCloud→LaserScan、短时局部障碍记忆、APF 跟随 | `/cmd_vel` | 轻量历史地图方案 |
| `go2_stereo_apf_follow` | 当前帧点云上的 APF 或带侧向锁定的 VFH | `/cmd_vel` | 低延迟局部方案 |
| `go2_uwb_dwb_follow` | 两点适配路径 + Nav2 DWB 局部控制 | `/cmd_vel` | DWB 对照方案 |
| `go2_uwb_mppi_follow` | 直线路径 + Nav2 MPPI + STVL + 目标人体点云清除 | `/cmd_vel` | MPPI 对照方案 |
| `go2_uwb_teb_follow` | rolling costmap 上 Smac Hybrid 规划 + TEB 跟踪 | `/cmd_vel` | 局部绕行方案 |
| `go2_exact_mppi_follow` | 直接把筛选点云送入 JAX EXACT-MPPI，不使用 Nav2 costmap | `/cmd_vel` | GPU 研究方案 |
| `behavior_ext_plugins` | `FollowPath` 恢复行为树执行器与自由空间方向 `BackUp` 插件 | Behavior action | Nav2 扩展 |

更详细的方案关系、选择依据和数据流见 [架构说明](docs/ARCHITECTURE.md)。

## 环境与编译

目标环境：Ubuntu 22.04、ROS 2 Humble、C++17、Python 3。

> UWB 驱动已按实机可用版本回退，重新使用 ARM64 厂商滤波静态库；`libAoa_robot_example` 需要在 ARM64 Go2 容器中编译。上层跟随节点仍直接使用驱动输出，不再额外做低通、跳变、状态、置信度或超时过滤。

```bash
source /opt/ros/humble/setup.bash
cd ~/go2_follow
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

当前工作区的可选运行依赖包括 RealSense ROS 驱动、RTAB-Map、`pointcloud_to_laserscan`、`spatio_temporal_voxel_layer`、外部 TEB 插件和 EXACT-MPPI/JAX。缺少某项时只编译需要的包：

```bash
colcon build --symlink-install \
  --packages-select uwb_aoa_pkg go2_dynamic_follow_avoidance go2_uwb_mppi_follow
```

当前主机上的实际构建、测试和 `rosdep` 结果见 [验证报告](docs/VALIDATION.md)。

## 通用启动准备

1. 启动 Go2 驱动、URDF 和机器人状态发布器。
2. 确认以下 TF 链存在。本仓库当前统一使用 `base_footprint`。

```text
odom -> base_footprint -> camera optical frame
```

3. 如需单独诊断 UWB，可运行下面命令；诊断结束后按 `Ctrl+C`，不要让它与会自动启动 UWB 的完整跟随 launch 同时占用串口：

```bash
ros2 run uwb_aoa_pkg libAoa_robot_example \
  /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AG00S82A-if00-port0 \
  --ros-args -p frame_id:=base_footprint -p publish_rate_hz:=10.0
```

4. 确认 D435i 已发布四个统一输入，并启动 RTAB-Map 局部障碍生成：

```bash
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/infra2/image_rect_raw
ros2 topic echo /camera/camera/infra1/camera_info --once
ros2 topic echo /camera/camera/infra2/camera_info --once
ros2 launch go2_dynamic_follow_avoidance d435i_rtabmap.launch.py
ros2 topic hz /local_grid_obstacle
ros2 run tf2_ros tf2_echo odom base_footprint
```

5. 从下列路线中只选择一条。示例：

```bash
# 自写局部 A* + Nav2 MPPI
ros2 launch go2_dynamic_follow_avoidance go2_follow_bringup.launch.py \
  start_uwb:=true

# 轻量 VFH
ros2 launch go2_stereo_apf_follow stereo_vfh_follow.launch.py \
  start_uwb:=true

# 直线路径 + Nav2 MPPI（默认自动启动 UWB）
ros2 launch go2_uwb_mppi_follow uwb_mppi_nav2.launch.py \
  use_sim_time:=false
```

## 文档索引

- [系统架构、方案比较和选择指南](docs/ARCHITECTURE.md)
- [优化建议与实施路线](docs/OPTIMIZATION.md)
- [构建、测试与依赖验证报告](docs/VALIDATION.md)
- [所有方案实机测试矩阵](docs/FOLLOW_TEST_MATRIX.md)
- [uwb_aoa_pkg](docs/packages/uwb_aoa_pkg.md)
- [go2_dynamic_follow_avoidance](docs/packages/go2_dynamic_follow_avoidance.md)
- [go2_uwb](docs/packages/go2_uwb.md)
- [jie_deamon](docs/packages/jie_deamon.md)
- [go2_stereo_apf_follow](docs/packages/go2_stereo_apf_follow.md)
- [go2_uwb_dwb_follow](docs/packages/go2_uwb_dwb_follow.md)
- [go2_uwb_mppi_follow](docs/packages/go2_uwb_mppi_follow.md)
- [go2_uwb_teb_follow](docs/packages/go2_uwb_teb_follow.md)
- [go2_exact_mppi_follow](docs/packages/go2_exact_mppi_follow.md)
- [behavior_ext_plugins](docs/packages/behavior_ext_plugins.md)

## 仓库目录

```text
src/Follow/                    go2_dynamic_follow_avoidance 包
src/behavior_ext_plugins/      Nav2 恢复行为插件
src/go2_exact_mppi_follow/     JAX EXACT-MPPI 方案
src/go2_stereo_apf_follow/     APF/VFH 方案
src/go2_uwb*/                  UWB + 不同局部控制器方案
src/jie/                       jie_deamon 短时局部地图方案
src/uwb/                       UWB 串口驱动与消息定义
third_party/                   EXACT-MPPI 固定版本说明
docs/                          工作区统一文档
```
