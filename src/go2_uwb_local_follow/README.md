# go2_uwb_local_follow

第一阶段的时间对齐、基线录制与验收见 [stage1_baseline.md](docs/stage1_baseline.md)。
第二阶段的目标运动估计、速度前馈、启停滞回及回退开关见 [stage2_target_motion.md](docs/stage2_target_motion.md)。
当前运行参数以 YAML 和运行时参数快照为准，下文早期阶段的示例数值不代表实际生效配置。

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

默认启用第二阶段：保留厂家消息源时间，按 odom 位姿将目标变换到统一坐标系，估计位置、速度和运动方向，再用人速前馈与距离修正跟随。目标过期 0.50 秒停车；跳点与重复源帧不续期。厂家 `state` 和 `pos_confidence` 只进入诊断，不阻断有限的 `x/y`。通过 `enable_target_estimation:=false` 回退到原有距离跟随。

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

UWB 名义转向使用 `/odom_leg.twist.twist.angular.z` 计算动态停止角：
`angle_deadband + |actual_wz| * turn_response_delay + actual_wz² /
(2 * angular_braking_accel)`。实际角速度越高越早撤销名义角速度；进入动态刹车区后
不会再执行角速度 P 补偿。再次转向仍使用 `angle_reengage` 滞回，并在实际角速度
高于 `angular_reverse_speed_threshold` 时禁止直接反向，以减少越过目标后的左右摆头。
动态刹车触发后会锁存零名义角速度，直到实测角速度低于
`angular_brake_release_speed`，防止停止角随速度下降后过早恢复同方向转向。

`local_velocity_planner_node` 读取 `/odom_leg` 的 `twist.twist.linear.x` 和
`twist.twist.angular.z` 作为当前真实速度，并缓存带时间戳的 pose，将障碍补偿到控制时刻。新增的 `rolling_obstacle_map_node` 独立
读取同一话题的带时间戳 pose：每帧 `/local_depth_observation` 先按观测时间戳插值
`odom -> base_footprint` 位姿并转换到局部 `odom` 二维体素地图，再把全部保留障碍补偿到
该观测时刻的当前 `base_footprint`，发布 `/local_rolling_obstacle` 给原规划器。
它不需要 SLAM、全局地图或全局路径。

滚动地图虽然保留障碍点的 `z` 用于输出，但占用单元、射线遍历和规划碰撞都只使用
`x/y`，所以它是二维地图。当前帧深度射线会清除相机与有效深度端点之间的历史
占用单元；当前帧确认的障碍会阻断射线并受到保护，射线末端保留 `0.10 m` 安全余量，
同一单元默认至少需要 `2` 条本帧射线穿过才清除。无效视差和量程外区域保持未知，
不会被当成自由空间。

时间衰减继续作为保守兜底：滚动地图默认只保留机器人周围 `3.0 m`、最近 `5.0 s`
内观测到的障碍；同一体素的新观测刷新时间，超时、超范围和超过点数上限的障碍
自动删除。里程计时间回退或短时间位置/朝向大跳变会立即清空历史地图。滚动地图
只在收到合法的新深度观测后发布，因此不会用历史点持续重发来掩盖双目断流；规划器
对其使用 `0.70 s` 超时。
规划器在该分支保留补偿后位于 `x >= -0.50 m` 的侧后方障碍，并关闭二次机身过滤，
避免真实历史障碍进入当前足迹后反而被当作机器人自身点删除。

每条候选轨迹仍从实测速度开始，按照加减速度限制展开，并在 `1.2 s` 预测时域
末尾继续追加到完全停止的制动尾段。速度采样、净空/TTC 分层和碰撞评分逻辑没有
因滚动地图而改变。

规划器默认从 UWB 名义线速度开始，按 `linear_speed_step=0.10 m/s` 逐档降速，
始终保留最低有效线速度 `min_linear_speed` 和停车档。例如名义速度为 `0.80 m/s`、
最低速度为 `0.23 m/s` 时，档位为 `[0.80, 0.70, 0.60, 0.50, 0.40, 0.30, 0.23, 0]`；
名义速度为 `0.30 m/s` 时为 `[0.30, 0.23, 0]`。非零步长至少为 `0.01 m/s`，
步长越小计算量越大。`linear_samples` 不控制这些分层档位。
设置 `linear_speed_step=0` 可恢复 `linear_speed_priority_scales` 旧比例模式。
固定步长模式下，诊断 `selected_speed_scale` 为选中速度与有效名义速度的实际比值。
每一层只
采样角速度并淘汰碰撞轨迹。无硬碰撞但未达到基础净空或保守 TTC 的轨迹记为
勉强安全，不参与本层评分并触发下一减速层；当前速度层存在严格安全轨迹时，
只在该层选择总代价最低的候选，不允许总代价偷选更低线速度。任何非零速度层
存在严格安全轨迹时都不会选择停车，只有全部非零层失败后才开放停车和严格安全
的原地旋转候选。UWB 名义角速度使用 `/odom_leg` 实测角速度进行 P 反馈修正，默认
`angular_velocity_tracking_kp=1.0`，同时保留小命令死区、反向停稳保护和最大角速度
限幅；不再叠加原角速度阻尼，也不强制抬升跟随角速度。主动避障候选继续使用独立
的最小角速度和 `1.50 rad/s` 最大角速度，
无障碍 UWB 跟随仍允许达到 `2.00 rad/s`，并在调整后重新预测碰撞轨迹。
`/odom_leg`、名义速度或障碍点云任一超时都会故障停车。
正前方紧急区默认需要连续 `3` 个新点云帧命中才锁存急停，连续 `3` 个新帧清空
才解除；同一帧不会因控制循环重复执行而被重复计数。确认期间普通轨迹碰撞检查
仍然有效，可通过 `emergency_confirm_frames` 调整确认帧数。急停锁存后立即发布零速，
实测线/角速度进入停稳门槛后直接以 `0.40 m/s` 直线倒退；当前帧确认触发障碍已经
离开急停区域后立即停车，并等待急停锁存解除。`0.20 m` 只作为区域始终未清空时的
最大命令距离兜底。倒退前和倒退过程中都会用增加 `0.05 m` 安全边界的足迹检查
侧后方滚动障碍；后方不安全、达到兜底距离或任一关键输入超时时只停车，不强行恢复。

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
- `EMERGENCY_BRAKING`：急停已确认，正在保持零速并等待底盘停稳。
- `EMERGENCY_REVERSING`：后向扫掠轨迹安全，正在直线倒退以退出急停区域。
- `EMERGENCY_REVERSE_BLOCKED`：侧后方轨迹不安全，禁止倒退并保持零速度。
- `EMERGENCY_ZONE_CLEARED`：触发障碍已经离开当前急停区域，停止倒退并等待锁存解除。
- `EMERGENCY_ZONE_REENTERED`：清空确认期间障碍再次进入急停区域，保持一周期零速后恢复倒退。
- `EMERGENCY_REVERSE_LIMIT_REACHED`：急停区域未清空但已达到最大后退预算，保持停车。
- `EMERGENCY_STOP`：未启用倒退恢复时的紧急停车状态。
- `BLOCKED`：所有候选轨迹碰撞，发布零速度。
- `NOMINAL_TIMEOUT` / `SENSOR_TIMEOUT` / `ODOM_TIMEOUT`：关键输入超时停车。
- `OBSTACLE_INVALID` / `ODOM_INVALID`：消息字段或坐标系不符合配置。


## 避障恢复与指令过渡预测

完整链路默认让规划器订阅 `/uwb/target_state`（odom 坐标的位置与速度），每周期转换到当前机身系。
单独启动规划器默认兼容原始模式：订阅 `target_topic`（默认 `/uwb/target_point`，`PointStamped`），只接受
`base_frame` 下的有限坐标及有效时间戳。目标缺失、过期或坐标系错误会停车；单独
启动规划器时也必须提供该话题。名义角速度可能被跟随控制器的刹车逻辑置零，
因此不以名义角速度代替目标方位。

状态为 `PLANNING`、`AVOIDING`、`RECOVERING`（隔离输出时带 `_DEBUG`）。
避障阶段仍由轨迹安全检查选择绕行速度，不设置固定的恢复限速。退出避障或发生
所选运动需要反向纠偏、撤销转向时进入恢复阶段，以进入前最终下发的线速度为上限，允许减速、
禁止提速；上限随实际下发的减速指令下降。重新进入避障时重新按安全轨迹选速。
方位误差和转向反向本身不触发零线速度，安全过渡允许边走边回正。转向建立后的避障起步仍通过
原有最低有效速度和加速度限幅执行。已移除 `recovery_max_speed` 参数。
安全速度上限低于最低有效速度时停车，不抬升到死区以上。底盘实际减速需要时间，
零指令并不代表即时停止，预测仍包括实测初始速度和响应延迟。

只有目标方位、历史及即将下发的转向指令、实测转向同时满足恢复门槛，且未限制
提速的名义跟随轨迹有充分净空，才按新的障碍时间戳累计确认。连续
`recovery_clear_observations` 次成功后恢复正常加速；重复帧不累计，条件失效立即
清零。诊断提供 `target_heading`、`target_age_sec`、`recovery_speed_cap`、
`recovery_clear_count`、`turn_unestablished`、`turn_mismatch_sec` 和 `recovery_reversing`。
`recovery_reversing` 表示选中的运动目标与当前运动反向，仅用于状态/诊断，
不等于停车。目标方位在另一侧但仍需沿原方向绕障时，以安全绕行候选为准。

转向失跟根据最终候选的首条实际指令判断。只有指令达到 `turn_command_threshold`，
实测转向不同向或小于 `turn_measured_threshold`，并持续达到
`max(turn_tracking_timeout, command_response_delay)` 才停止前进并重新检查转向轨迹。
默认观察窗口为 0.35 s；窗口内检测到失跟先禁止提速，但不直接停车。
减小指令或穿过转向零点会清除累计。瞬时换向不直接停车；
窗口内仍须通过包含响应延迟的碰撞检查。若底盘响应超出模型，应标定预测参数。
原 UWB 控制器根据目标方位和距离生成名义速度的规则保持不变。

每个候选使用真实上一条指令计算首条输出，后续周期继续用同一限幅函数更新指令；
运动预测从实测速度开始，经过 `command_response_delay` 和加减速度限制响应指令。
预测包括指令及实际运动均完全停止的制动尾段。选中候选的 `first_command` 直接
发布，普通规划输出不再在碰撞检查后另做限幅。绕障方向偏好仅参与评分，不冒充
真实历史指令。急停/故障的强制停车和专用倒退恢复仍保留各自的处理路径。

响应延迟 0.15 s、角响应比例 1.0 是可配置模型初值，不是 Go2 实测标定值。
转向不足保护可阻止“转不起来还向前加速”，但不能代替完整的响应标定或补回
已经被地图删除的侧面障碍。本次未修改历史地图清理策略。增加过渡预测和恢复
检查会增加规划耗时，实机需检查 `planning_time_ms` 是否满足控制周期。
