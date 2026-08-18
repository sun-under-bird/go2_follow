# 系统架构与方案选择

## 1. 工作区的真实结构

仓库可以分成四层：

```text
设备层
  uwb_aoa_pkg              D435i RealSense 驱动（仓库外）
       │                             │
目标与感知适配层                      │
  UWB 原始坐标解析/TF            RTAB-Map 局部障碍生成
       │                             │
规划与控制层                         │
  A* / APF / VFH / DWB / MPPI / TEB / EXACT-MPPI
       │
恢复与执行层
  FollowPath 恢复行为树 / 各算法停车逻辑
       │
  /cmd_vel -> Go2 底盘接口
```

这些控制包大多订阅同一个 UWB 消息和相似的点云，却实现了各自的解析、坐标转换和控制策略。因此它们是互斥的实验路线，不是串联模块。

## 2. 共同坐标和话题约定

ROS 平面约定：`x` 向前、`y` 向左、`z` 向上，正 `angular.z` 为逆时针左转。

建议统一：

| 语义 | 建议名称 | 当前仓库现状 |
| --- | --- | --- |
| 局部固定坐标 | `odom` | 基本一致 |
| 机器人控制坐标 | `base_footprint` | MPPI、APF、VFH、DWB、TEB 等统一使用 |
| UWB 目标坐标 | `base_footprint` | 当前对齐安装下驱动和目标适配器的统一坐标 |
| 原始 UWB | `/libAoa_robot_publisher` | 一致 |
| 左右红外图像 | `/camera/camera/infra1/image_rect_raw`、`/camera/camera/infra2/image_rect_raw` | D435i 已完成矫正 |
| 左右相机内参 | `/camera/camera/infra1/camera_info`、`/camera/camera/infra2/camera_info` | RealSense 驱动发布 |
| 障碍/地面点云 | `/local_grid_obstacle`、`/local_grid_ground` | RTAB-Map 从上述双目输入生成 |
| 里程计 | `/odom` | 所有包统一订阅 |
| 控制速度 | `/cmd_vel` | 所有控制与恢复行为直接输出；没有中间转发话题 |

必须确认：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint <pointcloud_frame>
```

## 3. 控制路线比较

| 路线 | 路径/规划 | 障碍表示 | 运动模型 | 优点 | 主要限制 |
| --- | --- | --- | --- | --- | --- |
| `go2_dynamic_follow_avoidance` | 自写二维 A*，MPPI 只跟踪 | 自写栅格 + Nav2 VoxelLayer 两套地图 | 默认差速 | 有显式局部绕路和失败恢复 | 仍存在两套障碍表示 |
| `go2_uwb` | 无路径，比例控制 | PCL ROI、最近距离、左右点数 | 差速转向 | 结构直观、C++ 实现 | 绕障策略简单、没有空间路径规划 |
| `jie_deamon` | 无路径，APF | LaserScan + 1 m 短时记忆 | 全向/差速可裁剪 | 能记住短时侧向障碍 | 局部地图插入复杂度高，仍受前视传感器限制 |
| `go2_stereo_apf_follow/APF` | 无路径，势场 | 当前点云 | 全向 | 延迟低、实现简单 | 局部极小值、急停后直接恢复 |
| `go2_stereo_apf_follow/VFH` | 极坐标方向选择 | 当前点云直方图 | 全向 | 有左右绕行锁定，抖动较小 | 不做空间路径规划，狭窄/凹形障碍仍可能失败 |
| `go2_uwb_dwb_follow` | 两点适配 path | Nav2 local costmap | 差速 | 标准 DWB + 空闲方向恢复 | 两点 path 不是真实规划，大障碍可能卡住 |
| `go2_uwb_mppi_follow` | 插值直线路径 | STVL + 目标人体清除 | 默认差速 | MPPI + 人体过滤 + 空闲方向恢复 | 没有全局绕路，只跟踪局部直线路径 |
| `go2_uwb_teb_follow` | Smac Hybrid | rolling global/local costmap + 人体清除 | 差速 | 局部主动绕路并带失败恢复 | 依赖外部 TEB/STVL，部署复杂度较高 |
| `go2_exact_mppi_follow` | 直线参考轨迹 | 直接选取最多 300 个点 | 全向 | 不依赖 Nav2 costmap，适合 GPU 研究 | JAX/固定第三方版本，缺少独立速度安全层 |

## 4. 如何选择

- 先验证 UWB 跟随方向：使用 `go2_dynamic_follow_avoidance` 的 simple 模式，保持底盘不消费 `/cmd_vel`，只观察话题。
- CPU 有限、目标环境开阔：优先 VFH；它比纯 APF 更能抑制左右绕行抖动。
- 需要在局部障碍间规划一条明确路径：使用自写 A* + MPPI。
- 已经维护 Nav2、希望比较标准控制器：选择 DWB 或直连 MPPI。
- 希望在 6 m rolling window 中绕过较大障碍：评估 Smac Hybrid + TEB。
- Jetson Orin 上做控制算法研究且能维护 JAX 环境：评估 EXACT-MPPI。

## 5. 启动组合规则

### 可以共享

- 一个 `uwb_aoa_pkg/libAoa_robot_example`。
- 一个 D435i 驱动和一个 RTAB-Map 局部障碍生成链路。
- 一个 Go2 里程计和 TF 树。
- RViz、PlotJuggler、rosbag 记录器。

### 必须互斥

- 任意两个跟随控制器都不能同时发布 `/cmd_vel`。
- DWB、MPPI、TEB/Dynamic 的 `controller_server`、`behavior_server` 和相同 action 名不能并行运行。
- 不要再启动拼接拆图、`image_proc` 或 `stereo_image_proc`；本仓库直接消费 D435i 已矫正话题。

本仓库按“一次只启动一个方案”设计，因此不再设置单纯的话题转发桥。若未来确实要在线切换控制器，应引入具备优先级、心跳和急停语义的正式仲裁器，而不是普通 relay。

## 6. 当前控制语义

当前阶段按要求移除了 UWB 状态/置信度/跳变/超时门控、点云与 odom 输入超时、速度低通和加速度限制。各路线收到第一帧输入后直接使用最近数据，最终命令直接发到 `/cmd_vel`。TF 与数值有限性检查仍保留，因为不做坐标变换或让 NaN 进入控制器无法产生有意义的速度。

障碍碰撞、急停和 Nav2 costmap 检查属于避障算法本体，仍然保留；Nav2 跟踪失败后可以进入行为树恢复。传感器停更不会自动停车，这是当前配置最重要的实机风险。

## 7. 推荐联调顺序

1. 只启动驱动，记录静止和移动条件下的 UWB 数据，确认 `state` 语义、角度单位和坐标方向。
2. 只启动 D435i，检查两路 `image_rect_raw`、CameraInfo 和相机到机身的 TF。
3. 不接底盘，启动所选控制器，人工移动 UWB 标签和障碍物，观察目标、状态、路径和速度。
4. 人工验证 UWB、点云、里程计停更时仍会复用旧数据，并准备外部急停；TF 失败帧应被拒绝且节点不能崩溃。
5. 架空或限速到 `0.1 m/s` 接入底盘，确认前后左右与旋转方向。
6. 逐步增加速度，使用 rosbag 保存 UWB、点云、TF、odom、path、status 和 cmd_vel。
