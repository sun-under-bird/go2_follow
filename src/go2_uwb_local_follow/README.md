# go2_uwb_local_follow

当前已实现四个可独立验收的阶段：

1. 双目视差与 `base_footprint` 障碍点云。
2. 厂家 UWB 原始消息适配与不带避障的纯跟随控制。
3. 名义轨迹预测、矩形足迹碰撞检查和紧急停车调试。
4. 使用 `/odom_leg` 实测初始速度的局部速度采样与碰撞规划。

```text
infra1/infra2 已校正图像
  -> stereo_image_proc/disparity_node
  -> /stereo/disparity
  -> stereo_obstacle_projector_node
  -> /local_grid_obstacle (base_footprint)
```

自定义节点直接抽样视差并按 `Z=fT/d` 反投影，不创建完整稠密点云。每个三维点按视差时间戳通过 TF 转换到 `base_footprint`，只保留 `0.10 <= z <= 0.50 m`，最后执行 5 cm 体素过滤。

每个二维网格需要可配置数量的当前帧深度点支持，且障碍簇默认至少包含 3 个
三维 26 邻域连通体素。当前配置每网格需要 6 个深度点，可删除支持点过少以及
不同高度在俯视平面误连接的小伪影。该过滤不保存历史帧，因此不会引入多帧确认延迟。

## 编译

```bash
cd /root/go2_follow_worktree
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select go2_uwb_local_follow
source install/setup.bash
```

## 启动

相机驱动和 `base_footprint -> camera_infra1_optical_frame` TF 必须已经存在。
完整实机链路还需要厂家 UWB 话题 `/libAoa_robot_publisher`、里程计
`/odom_leg` 和底盘 `/cmd_vel` 接收节点。

一键启动完整感知、跟随和局部避障链路：

```bash
ros2 launch go2_uwb_local_follow local_follow.launch.py
```

该启动文件默认只由局部速度规划器发布 `/cmd_vel`；UWB 跟随节点只发布
名义速度，不会绕过碰撞检查。架空调试时可禁止真实速度：

```bash
ros2 launch go2_uwb_local_follow local_follow.launch.py enable_motion:=false
```

以下独立启动命令仍用于分阶段验收。

```bash
ros2 launch go2_uwb_local_follow stereo_obstacle_cloud.launch.py
```

临时发布完整深度图用于检查：

```bash
ros2 launch go2_uwb_local_follow stereo_obstacle_cloud.launch.py \
  publish_debug_depth:=true
```

## 验证

```bash
ros2 topic hz /stereo/disparity
ros2 topic hz /local_grid_obstacle
ros2 topic echo /stereo/obstacle_diagnostics
ros2 topic echo /local_grid_obstacle --field header --once
ros2 run tf2_ros tf2_echo base_footprint camera_infra1_optical_frame
```

空场景下，`/local_grid_obstacle` 可以是零点的合法当前帧点云。视差有效样本不足、TF 失败或输入超过 0.60 秒未更新时，不会把感知故障误报成自由空间，诊断话题会报告对应错误。

## UWB 纯跟随阶段

本阶段不缓存目标队列，也不做时间插值。厂家消息没有 Header，适配节点在收到每一帧时赋本机时间戳；20 Hz 控制器只保存最新一帧并零阶保持，超过 0.50 秒立即停车。厂家 `state` 和 `pos_confidence` 只进入诊断，不阻断有限的 `x/y`。

默认速度输出是隔离话题 `/cmd_vel_follow`：

```bash
ros2 launch go2_uwb_local_follow uwb_follow_only.launch.py
```

查看目标、名义速度、限加速度输出和状态：

```bash
ros2 topic echo /uwb/target_point
ros2 topic echo /go2_uwb_local_follow/nominal_cmd
ros2 topic echo /cmd_vel_follow
ros2 topic echo /go2_uwb_local_follow/follow_diagnostics
```

完成架空或安全区域验收并确认 `/cmd_vel` 没有其他发布者后，才切换真实底盘输出：

```bash
ros2 launch go2_uwb_local_follow uwb_follow_only.launch.py \
  cmd_vel_topic:=/cmd_vel
```

UWB 角速度随目标方位误差连续增大，最大限制为 `2.00 rad/s`；线速度随目标
距离误差增大，并在方位角变大时使用平滑比例降速。有效非零线速度范围默认设为
`0.12~0.80 m/s`，避免 MCF 长时间接收无法形成步态的极小前进速度。线加速度和
减速度均为 `0.80 m/s²`；从静止加速到最高线速度理论约需 1 秒。目标丢失属于
安全事件，仍绕过普通减速过程并立即发布零速度。

## 轨迹预测与碰撞调试阶段

这一步先检查纯跟随生成的名义速度，不做多组速度采样。节点用运动学模型预测
未来 `1.2 s` 的轨迹，在每个轨迹点放置经过安全膨胀的旋转矩形足迹，并与
`/local_grid_obstacle` 的二维障碍点进行碰撞检查。正前方紧急区出现障碍、输入
超时、点云无效或名义轨迹碰撞时，规划结果立即变为零速度。

先分别启动双目障碍点云和 UWB 纯跟随，再启动隔离的碰撞调试：

```bash
ros2 launch go2_uwb_local_follow stereo_obstacle_cloud.launch.py
ros2 launch go2_uwb_local_follow uwb_follow_only.launch.py
ros2 launch go2_uwb_local_follow local_collision_debug.launch.py
```

默认 `enable_motion:=false`，因此 `/cmd_vel_avoidance` 始终为零；实际判定结果在
`/go2_uwb_local_follow/collision_checked_cmd`，不会接管真实底盘。检查以下话题：

```bash
ros2 topic echo /go2_uwb_local_follow/collision_diagnostics
ros2 topic echo /go2_uwb_local_follow/collision_checked_cmd
ros2 topic echo /go2_uwb_local_follow/evaluated_path
ros2 topic echo /cmd_vel_avoidance
```

诊断状态含义：

- `NOMINAL_TIMEOUT`：名义跟随速度缺失或超时。
- `WAIT_OBSTACLE` / `OBSTACLE_INVALID`：等待点云或点云格式、坐标系无效。
- `SENSOR_TIMEOUT`：障碍点云超时，按不安全处理。
- `EMERGENCY_STOP`：障碍进入机器人正前方紧急区。
- `NOMINAL_COLLISION`：名义轨迹上的膨胀足迹将发生碰撞。
- `CLEAR_DEBUG`：名义轨迹无碰撞；当前仍只允许调试输出。

建议在 RViz 中同时显示 `/local_grid_obstacle`（PointCloud2）和
`/go2_uwb_local_follow/evaluated_path`（Path），依次验证直行、左转、右转以及
障碍从足迹外进入足迹时的状态变化。通过这项几何验收后，使用下一节的完整局部
速度规划器测试多组 `(v, w)` 采样和评分。

## 完整局部速度规划阶段

UWB 名义转向使用角度滞回：目标方位进入 `angle_deadband` 后停止转向，
只有再次越过更大的 `angle_reengage` 才重新转向。这用于避免底盘最小有效
角速度较大时，机器人每次穿过目标方向就立即反转。

`local_velocity_planner_node` 订阅 `/odom_leg`（`nav_msgs/msg/Odometry`），但只读取
`twist.twist.linear.x` 和 `twist.twist.angular.z` 作为当前真实速度；不使用 odom
位姿、不累计轨迹，也不把障碍物转换到 odom。每条候选轨迹从该实测速度开始，
按照加减速度限制展开，并在 `1.2 s` 预测时域末尾继续追加到完全停止的制动尾段。

规划器按 `[1.0, 0.85, 0.70, 0.50, 0.0]` 分层尝试 UWB 名义线速度，每一层只
采样角速度并淘汰碰撞轨迹；当前速度层存在安全轨迹时立即返回，不允许总代价
偷选更低线速度。只有整层角速度都不安全时才进入下一减速层。UWB 跟随允许连续
小角速度；主动避障候选使用独立的最小角速度和 `1.50 rad/s` 最大角速度，
无障碍 UWB 跟随仍允许达到 `2.00 rad/s`，并在调整后重新预测碰撞轨迹。
`/odom_leg`、名义速度或障碍点云任一超时都会故障停车。
正前方紧急区默认需要连续 `2` 个新点云帧命中才锁存急停，连续 `2` 个新帧清空
才解除；同一帧不会因控制循环重复执行而被重复计数。确认期间普通轨迹碰撞检查
仍然有效，可通过 `emergency_confirm_frames` 调整确认帧数。

隔离验收时使用三个终端：

```bash
ros2 launch go2_uwb_local_follow stereo_obstacle_cloud.launch.py
ros2 launch go2_uwb_local_follow uwb_follow_only.launch.py enable_motion:=false
ros2 launch go2_uwb_local_follow local_velocity_planner.launch.py
```

规划器默认 `enable_motion:=false`，所以 `/cmd_vel_planned` 始终为零。实际采样结果
和限幅结果分别发布到：

```bash
ros2 topic echo /go2_uwb_local_follow/planned_cmd
ros2 topic echo /go2_uwb_local_follow/final_cmd
ros2 topic echo /go2_uwb_local_follow/planner_diagnostics
```

在 RViz 中显示 `/go2_uwb_local_follow/selected_path`。该 Path 包括规划时域以及
完整制动尾段，因此可能比原来的 `/evaluated_path` 更长。在线隔离测试中约评估
11～100 条去重候选；当前约 200 个障碍点时，紧急停车采样约 `7.9 ms`，完整
99 候选采样约 `25.1 ms`，均低于 20 Hz 控制周期的 `50 ms`。

诊断状态含义：

- `PLANNING_DEBUG`：输入正常，正在隔离规划。
- `PLANNING`：输入正常且已经允许实机输出。
- `AVOIDING_DEBUG` / `AVOIDING`：名义轨迹进入障碍影响区，正在按线速度优先级绕障。
- `EMERGENCY_STOP`：正前方紧急区障碍达到连续帧确认条件，立即撤销线速度。
- `BLOCKED`：所有候选轨迹碰撞，发布零速度。
- `NOMINAL_TIMEOUT` / `SENSOR_TIMEOUT` / `ODOM_TIMEOUT`：关键输入超时停车。
- `OBSTACLE_INVALID` / `ODOM_INVALID`：消息字段或坐标系不符合配置。
