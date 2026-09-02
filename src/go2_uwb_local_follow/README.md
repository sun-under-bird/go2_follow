# go2_uwb_local_follow

当前已实现五个可独立验收的阶段：

1. 双目视差与 `base_footprint` 障碍点云。
2. 厂家 UWB 原始消息适配与不带避障的纯跟随控制。
3. 名义轨迹预测、矩形足迹碰撞检查和紧急停车调试。
4. 使用 `/odom_leg` 实测初始速度的局部速度采样与碰撞规划。
5. 使用点云时间戳、`/odom_leg` 位姿补偿、时间衰减和深度射线清除的滚动局部障碍地图。

```text
infra1/infra2 已校正图像
  -> stereo_image_proc/disparity_node
  -> /stereo/disparity
  -> stereo_obstacle_projector_node
  -> /local_grid_obstacle (过滤后的障碍点，兼容与调试输出)
  -> /local_depth_observation (障碍点 + 射线端点 + 相机视点)
  -> rolling_obstacle_map_node + /odom_leg pose
  -> /local_rolling_obstacle (当前点云时刻 base_footprint)
```

自定义节点直接抽样视差并按 `Z=fT/d` 反投影，不创建完整稠密点云。每个三维点按
视差时间戳通过 TF 转换到 `base_footprint`：`0.10 <= z <= 0.50 m` 的点参与障碍
过滤；`-0.10 <= z <= 0.50 m` 的有效深度点还可作为自由空间射线端点，因此低于
障碍高度阈值的地面观测也能清除旧障碍。障碍点最后执行 5 cm 体素过滤。

每个二维网格需要可配置数量的当前帧深度点支持，且障碍簇默认至少包含 3 个
三维 26 邻域连通体素。当前配置每网格需要 6 个深度点，可删除支持点过少以及
不同高度在俯视平面误连接的小伪影。该过滤不保存历史帧，因此不会引入多帧确认延迟。

## 编译

```bash
cd /home/bird/go2_follow_rolling_map
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to go2_uwb_local_follow
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
ros2 launch go2_uwb_local_follow rolling_obstacle_map.launch.py
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
ros2 topic hz /local_depth_observation
ros2 topic hz /local_rolling_obstacle
ros2 topic echo /stereo/obstacle_diagnostics
ros2 topic echo /go2_uwb_local_follow/rolling_map_diagnostics
ros2 topic echo /local_depth_observation --field header --once
ros2 run tf2_ros tf2_echo base_footprint camera_infra1_optical_frame
```

空场景下，`/local_grid_obstacle` 可以是零点的合法当前帧点云。只有视差有效的深度点
才产生射线；无效视差仍视为未知空间，不能清除障碍。视差有效样本不足、TF 失败或
输入超过 0.60 秒未更新时，不会把感知故障误报成自由空间，诊断话题会报告对应错误。

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

`local_velocity_planner_node` 仍只读取 `/odom_leg` 的 `twist.twist.linear.x` 和
`twist.twist.angular.z` 作为当前真实速度。新增的 `rolling_obstacle_map_node` 独立
读取同一话题的带时间戳 pose：每帧 `/local_depth_observation` 先按观测时间戳插值
`odom -> base_footprint` 位姿并转换到局部 `odom` 二维体素地图，再把全部保留障碍补偿到
该观测时刻的当前 `base_footprint`，发布 `/local_rolling_obstacle` 给原规划器。
它不需要 SLAM、全局地图或全局路径。

滚动地图虽然保留障碍点的 `z` 用于输出，但占用单元、射线遍历和规划碰撞都只使用
`x/y`，所以它是二维地图。当前帧深度射线会清除相机与有效深度端点之间的历史
占用单元；当前帧确认的障碍会阻断射线并受到保护，射线末端保留 `0.10 m` 安全余量，
同一单元默认至少需要 `2` 条本帧射线穿过才清除。无效视差和量程外区域保持未知，
不会被当成自由空间。

时间衰减继续作为保守兜底：滚动地图默认只保留机器人周围 `3.0 m`、最近 `1.0 s`
内观测到的障碍；同一体素的新观测刷新时间，超时、超范围和超过点数上限的障碍
自动删除。里程计时间回退或短时间位置/朝向大跳变会立即清空历史地图。滚动地图
只在收到合法的新深度观测后发布，因此不会用历史点持续重发来掩盖双目断流；规划器
对其使用 `0.70 s` 超时。
规划器在该分支保留补偿后位于 `x >= -0.50 m` 的侧后方障碍，并关闭二次机身过滤，
避免真实历史障碍进入当前足迹后反而被当作机器人自身点删除。

每条候选轨迹仍从实测速度开始，按照加减速度限制展开，并在 `1.2 s` 预测时域
末尾继续追加到完全停止的制动尾段。速度采样、净空/TTC 分层和碰撞评分逻辑没有
因滚动地图而改变。

规划器按 `[1.0, 0.85, 0.70, 0.50, 0.0]` 分层尝试 UWB 名义线速度，每一层只
采样角速度并淘汰碰撞轨迹。无硬碰撞但未达到基础净空或保守 TTC 的轨迹记为
勉强安全，不参与本层评分并触发下一减速层；当前速度层存在严格安全轨迹时，
只在该层选择总代价最低的候选，不允许总代价偷选更低线速度。任何非零速度层
存在严格安全轨迹时都不会选择停车，只有全部非零层失败后才开放停车和严格安全
的原地旋转候选。UWB 跟随允许连续
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
ros2 topic echo /go2_uwb_local_follow/rolling_map_diagnostics
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
