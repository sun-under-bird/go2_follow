# go2_uwb_local_follow

当前已实现五个可独立验收的阶段：

1. 双目视差与 `base_footprint` 障碍点云。
2. 厂家 UWB 原始消息适配与不带避障的纯跟随控制。
3. 名义轨迹预测、矩形足迹碰撞检查和紧急停车调试。
4. 使用 `/leg_odom2` 实测初始速度的 MPPI 时变控制序列与碰撞规划。
5. 使用点云时间戳、`/leg_odom2` 位姿补偿、时间衰减和深度射线清除的滚动局部障碍地图。

```text
infra1/infra2 已校正图像
  -> stereo_image_proc/disparity_node
  -> /stereo/disparity
  -> stereo_obstacle_projector_node
  -> /local_grid_obstacle (过滤后的障碍点，兼容与调试输出)
  -> /local_depth_observation (障碍点 + 射线端点 + 相机视点)
  -> rolling_obstacle_map_node + /leg_odom2 pose
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
cd /home/cat/robot_ws/go2_follow_develop
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to go2_uwb_local_follow
source install/setup.bash
```

## 启动

相机驱动和 `base_footprint -> camera_infra1_optical_frame` TF 必须已经存在。
完整实机链路还需要厂家 UWB 话题 `/libAoa_robot_publisher`、里程计
`/leg_odom2` 和底盘 `/cmd_vel` 接收节点。

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

速度输出默认直接接底盘 `/cmd_vel`。⚠️ Lite3 上 `/cmd_vel` 会先过
`lite3_twist_bridge`（10 Hz 补流 + 自动切 Vision/Joystick 模式）再由 `jetson2motion`
送到狗，**启动前必须确认 `/cmd_vel` 上没有其他发布者**（键盘 teleop 也发这个话题）。
首次验收请显式关掉速度输出，只看诊断：

```bash
ros2 launch go2_uwb_local_follow uwb_follow_only.launch.py enable_motion:=false
```

查看目标、名义速度、限加速度输出和状态：

```bash
ros2 topic echo /uwb/target_point
ros2 topic echo /go2_uwb_local_follow/nominal_cmd
ros2 topic echo /cmd_vel
ros2 topic echo /go2_uwb_local_follow/follow_diagnostics
```

架空或安全区域验收通过、且确认 `/cmd_vel` 没有其他发布者后，才显式打开真实底盘输出：

```bash
ros2 launch go2_uwb_local_follow uwb_follow_only.launch.py \
  enable_motion:=true
```

UWB 角速度随目标方位误差连续增大，最大限制为 `2.00 rad/s`；线速度随目标
距离误差增大，在方位角超过约 `28.6°` 后平滑降速，超过约 `80.2°` 才停止前进
并原地转向。有效非零线速度范围默认设为
`0.23~0.80 m/s`，避免 MCF 长时间接收无法形成步态的极小前进速度。线加速度和
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

UWB 名义转向使用 `/leg_odom2.twist.twist.angular.z` 计算动态停止角：
`angle_deadband + |actual_wz| * turn_response_delay + actual_wz² /
(2 * angular_braking_accel)`。实际角速度越高越早撤销名义角速度；进入动态刹车区后
不会再执行角速度 P 补偿。再次转向仍使用 `angle_reengage` 滞回，并在实际角速度
高于 `angular_reverse_speed_threshold` 时禁止直接反向，以减少越过目标后的左右摆头。
动态刹车触发后会锁存零名义角速度，直到实测角速度低于
`angular_brake_release_speed`，防止停止角随速度下降后过早恢复同方向转向。

`local_velocity_planner_node` 仍只读取 `/leg_odom2` 的 `twist.twist.linear.x` 和
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

新增障碍必须在相邻两帧落入同一 `odom` 体素才会进入正式地图；单帧缺失会清空候选
计数，已经确认的障碍则由后续观测直接刷新。时间衰减继续作为保守兜底：滚动地图
默认只保留机器人周围 `3.0 m`、最近 `5.0 s` 内观测到的障碍，超时、超范围和超过
点数上限的障碍自动删除。里程计时间回退或短时间位置/朝向大跳变会立即清空正式地图
和待确认候选。滚动地图
只在收到合法的新深度观测后发布，因此不会用历史点持续重发来掩盖双目断流；规划器
对其使用 `0.70 s` 超时。
点云、里程计和名义速度均同时检查源时间戳与本机单调接收时间；重复、积压或乱序
消息不能刷新有效期。点云先到而对应里程计稍后到达时，滚动地图保留待处理帧并在
后续里程计/看门狗周期重试，成功输出仍沿用原始采集时间戳。规划器在每个控制周期
把该采集时刻的障碍再次补偿到当前机身坐标。点云年龄超过 `0.30 s` 后线性降低线、
角速度上限，到 `0.70 s` 停车；新鲜点云恢复后无需重启即可重新规划。
规划器在该分支保留补偿后位于 `x >= -0.75 m` 的侧后方障碍，并关闭二次机身过滤，
避免真实历史障碍进入当前足迹后反而被当作机器人自身点删除。

普通局部规划已经改为自研 MPPI：每条候选包含整个预测时域内逐时刻变化的
`linear.x/angular.z` 控制序列，而不是一个固定速度圆弧。优化器将上一周期最优序列
前移作为暖启动，在其附近叠加一阶相关高斯噪声，并保留名义、停车和连续左右转弯
种子；每轮对无碰撞序列按总代价进行指数加权，默认执行 `48` 条、`2` 轮更新。
普通规划仍禁止负线速度，急停倒退继续由独立状态机负责。

每条序列从 `/leg_odom2` 实测速度开始，展开过程直接包含线/角加速度限制和 Go2
`0.25 m/s` 线速度执行死区，并在预测时域末尾追加到完全停止的制动尾段。硬碰撞
序列直接淘汰；运动序列还必须满足基础净空和保守 TTC，只有全部安全运动序列失败
后才允许选择停车。MPPI 加权平均后的序列会再次执行独立碰撞校验，若平均结果失去
可行性则退回本周期代价最低的已验证序列。
障碍点每周期只构建一次包围盒空间索引，名义序列、随机样本及最终均值序列共享该
索引；索引仅使用保守距离下界跳过无关区域，叶节点仍计算旋转矩形足迹精确净空。

当前版本仍直接使用滚动障碍点云，没有增加 OccupancyGrid、距离场、A* 或局部路径
引导，因此 MPPI 提升的是时变控制能力，不保证单独解决所有 U 型拓扑陷阱。
UWB 名义角速度使用 `/leg_odom2` 实测角速度进行 P 反馈修正，默认
`angular_velocity_tracking_kp=1.0`，同时保留小命令死区、反向停稳保护和最大角速度
限幅；不再叠加原角速度阻尼，也不强制抬升跟随角速度。普通 MPPI 控制序列使用
`1.50 rad/s` 最大角速度，并在调整后重新预测碰撞轨迹。
`/leg_odom2`、名义速度或障碍点云任一超时都会故障停车。
正前方紧急区默认需要连续 `3` 个新点云帧命中才锁存急停，连续 `3` 个新帧清空
才解除；同一帧不会因控制循环重复执行而被重复计数。确认期间普通轨迹碰撞检查
仍然有效，可通过 `emergency_confirm_frames` 调整确认帧数。急停锁存后立即发布零速，
实测线/角速度进入停稳门槛后直接以 `0.30 m/s` 直线倒退；当前帧确认触发障碍已经
离开急停区域后立即停车，并等待急停锁存解除。`0.40 m` 只作为区域始终未清空时的
最大命令距离兜底。倒退前和倒退过程中都会用增加 `0.05 m` 安全边界的足迹检查
侧后方滚动障碍；后方不安全、达到兜底距离或任一关键输入超时时只停车，不强行恢复。

隔离验收时使用三个终端：

```bash
ros2 launch go2_uwb_local_follow stereo_obstacle_cloud.launch.py
ros2 launch go2_uwb_local_follow uwb_follow_only.launch.py enable_motion:=false
ros2 launch go2_uwb_local_follow local_velocity_planner.launch.py
```

规划器在这个 launch 里默认 `enable_motion:=false`，所以它只往 `/cmd_vel` 发零速度。
⚠️ 这也意味着它是 `/cmd_vel` 的一个发布者：跑这一路时不要再开键盘 teleop，
否则两路指令在同一条话题上互相覆盖。MPPI 首个控制量和限幅结果分别发布到：

```bash
ros2 topic echo /go2_uwb_local_follow/planned_cmd
ros2 topic echo /go2_uwb_local_follow/final_cmd
ros2 topic echo /go2_uwb_local_follow/planner_diagnostics
ros2 topic echo /go2_uwb_local_follow/rolling_map_diagnostics
```

在 RViz 中显示 `/go2_uwb_local_follow/selected_path`。该 Path 包括规划时域以及
完整制动尾段，因此可能比原来的 `/evaluated_path` 更长。默认每周期评估
`48 × 2 = 96` 条时变控制序列；`planning_time_ms` 用于现场核对是否稳定低于
20 Hz 控制周期的 `50 ms`。当前机器的固定种子侧墙基准中，5000 个障碍点的中位
耗时约 `1.5 ms`、P95 约 `1.6 ms`；实机仍须结合目标 CPU 和真实点云复测。

诊断状态含义：

- `PLANNING_DEBUG`：输入正常，正在隔离规划。
- `PLANNING`：输入正常且已经允许实机输出。
- `AVOIDING_DEBUG` / `AVOIDING`：名义轨迹进入障碍影响区，正在优化时变控制序列。
- `EMERGENCY_BRAKING`：急停已确认，正在保持零速并等待底盘停稳。
- `EMERGENCY_REVERSING`：后向扫掠轨迹安全，正在直线倒退以退出急停区域。
- `EMERGENCY_REVERSE_BLOCKED`：侧后方轨迹不安全，禁止倒退并保持零速度。
- `EMERGENCY_ZONE_CLEARED`：触发障碍已经离开当前急停区域，停止倒退并等待锁存解除。
- `EMERGENCY_ZONE_REENTERED`：清空确认期间障碍再次进入急停区域，保持一周期零速后恢复倒退。
- `EMERGENCY_REVERSE_LIMIT_REACHED`：急停区域未清空但已达到最大后退预算，保持停车。
- `EMERGENCY_STOP`：未启用倒退恢复时的紧急停车状态。
- `BLOCKED`：所有候选轨迹碰撞，发布零速度。
- `NOMINAL_STOP`：上游显式要求零速，优先停止且不进入倒退恢复。
- `WAIT_TIME_ALIGNED_ODOM`：点云对应位姿尚未到达，停车并等待后续里程计自动重试。
- `WAIT_FRESH_RECOVERY_OBSERVATION`：旧地图只用于停车，等待新点云后再恢复主动动作。
- `NOMINAL_TIMEOUT` / `SENSOR_TIMEOUT` / `ODOM_TIMEOUT`：关键输入超时停车。
- `OBSTACLE_INVALID` / `ODOM_INVALID`：消息字段或坐标系不符合配置。
