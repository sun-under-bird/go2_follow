# 第二阶段：目标运动估计与速度前馈

本阶段在第一阶段的障碍时间对齐之上增加人的位置、速度、运动方向估计，以及跟随启停滞回。
目标估计由 `uwb_follow_controller_node` 维护，控制器和规划器共用 `/uwb/target_state`。
地图占用、人体与墙的区分、路线参考和后退状态机留到后续阶段；最新障碍检查仍由原规划器执行。

## 启动和回退

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to go2_uwb_local_follow --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

# 先检查数据、状态和名义规划输出；不向底盘发非零速度。
ros2 launch go2_uwb_local_follow local_follow.launch.py \
  enable_target_estimation:=true enable_motion:=false
```

完整链路默认开启第二阶段。与第一阶段对照时，将同一条命令改为
`enable_target_estimation:=false`：它同时关闭目标估计/前馈/行走滞回，使规划器恢复原始
`PointStamped` 目标，并让适配器恢复赋接收时间。障碍时间补偿及第一阶段记录功能仍开启。
这个开关在启动时生效，运行中修改参数不会切换算法；应分别重启、录制两组实验。

`uwb_follow_only.launch.py` 同样支持开关，默认开启，但该链路不带避障。
单独启动 `local_velocity_planner.launch.py` 默认关闭，以兼容已有原始目标输入；若与新跟随
节点搭配，要显式设置 `enable_target_estimation:=true`。直接启动可执行文件时，对应节点参数
分别是控制器 `enable_target_estimation`、规划器 `use_target_state`、适配器 `preserve_source_stamp`。

完整 launch 统一传递 `target_state_topic`、`target_prediction_sec`、`odom_frame` 和机身坐标系，
避免两端目标预测不同。跟随参数在 `config/uwb_follow_only.yaml`，规划参数在
`config/local_velocity_planner.yaml`。launch 中的同名参数会覆盖 YAML。

## 坐标、时间与失效处理

1. 适配器保留厂家消息 Header 的时间戳，再应用原有安装外参；有限的 `x/y` 才会转发。
   当前厂家驱动在**发布消息时**打时间戳，不是硬件采样时间，厂家内部滤波/传输延迟仍需实测。
   `state` 与 `pos_confidence` 继续用于诊断，没有臆造未知状态码的拒绝规则。
2. 控制器按 UWB 源时间查找 odom 位姿，将机身目标转到连续的 odom 坐标系。
   默认位姿缓存 3 s、最大插值间隙 0.20 s、最大外推 0.05 s，与第一阶段一致。
3. 用最近 0.40 s 的位置进行时间回归，再对速度作 0.12 s 时间常数滤波。
   至少 3 个样本且跨度达到 0.18 s 才输出有效估计，因此源数据应稳定达到约 10 Hz 或更高。
   位移门限为 `0.20 m + 3.0 m/s × Δt`，预测创新门限为 0.45 m，超出视为跳点。
4. `/uwb/target_state` 使用 `nav_msgs/Odometry`：Header 保留最后接受的 UWB **源时间**，
   `header.frame_id` 与 `child_frame_id` 均为 odom，pose 是人的世界位置，twist 是世界速度。
   orientation 表示最近确认的**运动方向**，不是视觉测得的身体朝向；首次确认前为单位四元数，
   是否有效见 `direction_valid`。停止时保留最近方向。该回归不估计统计协方差，对角线统一填
   `1e6`，不能直接作为高置信定位输入融合。
5. 每个控制周期按人的速度预测最多 0.20 s，并用当前 odom 转回机身系；控制器与规划器执行
   相同变换。目标最多有效 0.50 s，位置预测在 0.20 s 后封顶，断流不会无限外推。
6. 零/过期/过远未来时间戳被拒绝；重复、乱序、跳点不会刷新有效目标的接收看门狗。
   连续 0.60 s 没有可接受样本，重新积累目标窗口。odom 跳变、ROS 时钟回退会清空估计、
   行走状态和相关快照。重置前的旧源帧不能重新进入新坐标区间。
   控制位姿缺失、目标或 odom 失效时输出零速度；ROS 源年龄与 steady clock 断流看门狗同时检查。

## 行为与速度

`human_motion` 输出 `UNKNOWN / FORWARD / SLOWING / STOPPED / TURNING / RETURNING`。
停止/移动阈值默认 0.08/0.16 m/s；将当前运动方向与最近 1.50 s 中较早的可靠方向比较，
0.45 rad 以上视作转弯，2.40 rad 以上视作返回。方向与行为变化均需连续 3 次有效更新确认。
这是短时运动事件：完成掉头并沿新方向稳定走一段时间后会恢复 `FORWARD`；长时间停止后重新
起步没有可靠旧方向时，也不会强行标成掉头。

名义前进速度为：

```text
有符号人速前馈 = 人在 odom 中的速度投影到当前机器人→人的方向
期望速度 = (人速前馈 + linear_kp × (距离 - follow_distance)) × 方位降速系数
```

使用有符号投影，迎面返回时不会因为“速度大小很大”而加速追人。前馈小于等于
`-0.15 m/s` 时停止名义前进；侧后方仍使用原有盲转及动态角速度制动。
行为标签用于观测和解释，前进抑制直接使用连续估计的有符号速度，不等待标签再多确认一轮。

- 距离超过期望值 0.18 m，或处于合适距离且前馈支持有效迈步，才开始启动确认。
- 距离不大于期望值 +0.06 m 且期望速度不大于 0.15 m/s，开始停止确认。
- 确认至少跨 0.15 s、至少两个不同源样本；同一 UWB 帧上的多个控制周期不累计确认。
- 停下后至少保持 0.30 s。距离过近、迎面接近或盲转可立即撤销行走状态。
- 近距离阈值默认 `follow_distance - 0.05 m`，再加实测相对接近速度所需的制动距离
  `closing_speed² / (2 × max_linear_decel)`。同速行走不额外增加制动余量。
- 行走目标只输出零或 `[0.23, 0.80] m/s`。继续使用原有加减速限制和避障规划；撤销行走
  不等于底盘已经物理停稳，滤波、传输和底盘延迟仍影响最小距离。

低于底盘最低有效速度的持续慢走无法同时做到恒定距离和连续迈步。本实现用距离带减少
启停次数，不通过持续发送无效小速度假装平滑。目标人本身仍参与现有障碍处理；靠近人体
造成的规划停车需要第三阶段进一步解决，不能通过删除障碍点绕过。

## 记录和验收

第一阶段 `record_baseline.py` 已默认加录 `/uwb/target_state`。`follow_cycle` 新增：
源时间/年龄/有效性、`human_x/y_odom`、`human_vx/vy_odom`、确认方向、行为、跳点计数、
`feedforward_v`、`desired_v`、`walking` 和 `walk_reason`。
`target_update` 可区分 `WARMING / ACCEPTED / OUTLIER / DUPLICATE / INVALID_SOURCE_TIME / NO_SOURCE_POSE`。
`control_cycle.use_target_state` 用于确认规划器采用哪一组输入。

沿用第一阶段录制与分析命令，并在场景备注记录第二阶段开关。分析报告中的 `distance` 是
**控制器使用的目标距离估计**，新旧两组分别经过不同处理；不能仅凭这个字段变平滑认定实际
误差下降。应同时核对两组原始 `/uwb/target_point`、实测 odom 和独立距离测量，源延迟也要一致。
单独回放用于比较决策，只有闭环实机对照才能验证实际跟随距离。

固定场景每组至少三回合：

| 场景 | 检查 |
| --- | --- |
| 0.3/0.5/0.7 m/s 匀速走 | 人速估计、距离误差、启动延迟、饱和时间 |
| 0.10/0.15 m/s 慢走 | 距离带、实测启停次数、没有持续无效小速度 |
| 突然停止并静止 10 s | 停车延迟、最小距离、停下后不反复迈步 |
| 正常转弯、开阔区掉头迎面返回 | 行为确认时间、方向无单帧翻转、迎面时不继续名义前进 |
| 静止人、机器人平移/转动 | odom 中的人速接近零，不把机器人转向误认为人的运动 |
| 跳点、延迟、断流、重放重复包、odom 重置 | 拒绝原因、有效样本不续期、停车和重新确认 |
| 行人突然横穿、现有墙角场景 | 最新障碍仍触发制动/让行，原有安全净空不退化 |

离线自动测试覆盖回归估计、异常包、行为确认、启停、慢走及闭环一维仿真；ROS 测试串联实际
适配器/控制器/规划器并使用独立 domain、localhost 和 `/test/*` 输出。执行：

```bash
colcon test --packages-select go2_uwb_local_follow
colcon test-result --verbose
```

一维理想模型中，人以 0.5 m/s 走 15 s 后停止，默认期望距离 1 m，使用原有速度变化限制：
10–15 s 匀速区间的平均绝对距离误差，旧公式约 0.888 m，新公式约 0.025 m；
停止后不再次启动，最终距离约 0.847 m。该模型不含传感器噪声、避障限制及底盘响应延迟，
结果仅证明控制逻辑可消除比例跟随的主要稳态落后，**不作为实机距离或制动验收结果**。
