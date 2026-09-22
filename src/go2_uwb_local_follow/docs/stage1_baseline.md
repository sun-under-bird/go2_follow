# 第一阶段：基线记录和障碍时间对齐

本阶段增加基线记录，修正数据时效与障碍坐标对齐，保留跟随速度公式、速度档评分、恢复和后退状态机。
高点数检查发现原碰撞循环的重复三角函数与逐点 hypot 开销过大，因此同时进行了等价计算优化：
每个位姿只计算一次旋转，比较距离平方，最终只开一次平方根；碰撞足迹和净空定义保持不变。
实际基线与墙体稳定性必须在目标机器人采集；开发机上的测试不能替代实机验收。

## 启动与对照

先编译并加载环境：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to go2_uwb_local_follow --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

记录旧时间处理方式的对照组（保留逐周期记录）：

```bash
ros2 launch go2_uwb_local_follow local_follow.launch.py \
  enable_motion:=false compensate_obstacle_motion:=false enforce_source_time:=false \
  enable_cycle_telemetry:=true
```

修正组：

```bash
ros2 launch go2_uwb_local_follow local_follow.launch.py \
  enable_motion:=false compensate_obstacle_motion:=true enforce_source_time:=true \
  enable_cycle_telemetry:=true
```

`local_velocity_planner.launch.py` 也支持这三个开关。先隔离检查，再在固定场景中以
`enable_motion:=true` 做闭环对照。不要同时启动两组底盘输出。单独修改 YAML 也可以，
但上述两个 launch 的同名参数会覆盖 YAML，应优先通过 launch 设置对照模式。

对照模式恢复旧的“接收时间看门狗 + 不补偿到控制时刻”，不是旧提交的逐字节重现：
两组共同保留重复帧去重、位姿插值间隙限制、点云结构校验及后退检查范围修复。
需要严格旧版本对照时，应记录旧提交本身，并保存实际参数和版本；旧版本没有新增遥测，
不能用新增分析报告填补其未记录的字段。

`enable_cycle_telemetry:=false` 关闭新增逐周期/状态事件/控制障碍发布，保留原低频诊断。
在相同输入下比较开启/关闭时的外部 CPU 和定时统计，检查记录开销。

## 录制

在已启动的链路旁执行，被动录制工具不会发布运动指令：

```bash
ros2 run go2_uwb_local_follow record_baseline.py \
  --output /tmp/go2-baselines/left_corner_before_01 \
  --scenario left_corner --duration 60 --repo "$PWD" \
  --include-images --notes 'before; 此处写实际启动命令、墙角尺寸、行走速度'
```

`--duration 0` 持续录制直到 Ctrl-C。输出目录必须不存在。输出包括：

- `manifest.json`：场景、记录时间、Git 提交/工作区状态、参数获取结果、话题图、录制命令和 bag 信息。
- `working_tree.patch`：已跟踪文件的未提交改动；未跟踪文件需另行归档，不能仅依赖此补丁复现。
- `parameters/*.yaml`：运行节点的实际参数；缺失节点或参数读取失败明确保存在 manifest。
- `bag/`：原始 UWB、目标、odom、TF、障碍/射线/历史地图、速度、路径、诊断和新增遥测。
- `recorder.log`：录制进程输出，用于检查订阅、磁盘及丢消息告警。

`--include-images` 加录双目矫正图像、CameraInfo 和视差，支持之后从感知输入复现。
不加该参数仍能从深度观测或障碍点云回放，但不能复现 BM 深度生成。
定制话题/节点名使用可重复的 `--topic /实际话题 --node /实际节点`。
录制期间不要变更参数；如必须更改，应另开实验，`/parameter_events` 用于核对。

```bash
ros2 run go2_uwb_local_follow analyze_baseline.py \
  /tmp/go2-baselines/left_corner_before_01/bag \
  --output /tmp/go2-baselines/left_corner_before_01/report \
  --follow-distance 1.0 --control-period-ms 50
```

输出 `cycles.csv` 和 `summary.json`：距离绝对误差与 RMSE、实测启停次数、状态进入次数、
后退次数、感知源年龄、补偿/规划/整个回调耗时的均值/P50/P95/P99/最大值、超周期次数、
遥测序号缺口及录制话题频率。启停统计使用实测速度 0.10/0.04 m/s 滞回与 0.20 s 确认。
距离是 UWB 估计值，不是外部真值；`min_clearance` 是预测净空，不是实测机身侧间距。
报告不会把缺失/无穷数据写成零。目标新鲜度与 odom 启停统计分别按默认 0.5/0.1 s 筛选，
若修改这两个超时参数，应同步调整分析口径。

## 新增话题和时间语义

| 话题 | 内容 |
| --- | --- |
| `/go2_uwb_local_follow/control_cycle` | 每个控制周期的 DiagnosticArray：源时间、年龄、目标距离/方位、输入/候选/下发/实测速度、状态、原因、急停计数及耗时 |
| `/go2_uwb_local_follow/state_transitions` | 状态切换时立即发布，同样携带周期字段和 previous_state，不受 2 Hz 诊断节流影响 |
| `/go2_uwb_local_follow/follow_cycle` | 跟随控制的距离保持、盲转、动态角度制动状态；名义节点隔离输出时仍能看到控制原因 |
| `/go2_uwb_local_follow/map_observations` | 每帧有效地图更新的源时间、处理耗时和地图/射线统计 |
| `/go2_uwb_local_follow/control_obstacles` | 该控制周期实际参与碰撞检查的障碍，Header 是控制时刻，源观测时间见同周期 control_cycle |

`/local_rolling_obstacle` 仍是**观测时刻**的 `base_footprint`，Header 保留图像源时间；
新增 `last_seen` 和 `expires_at` 两个 FLOAT64 字段（ROS 时间，单位秒），分别描述每个体素的
最后真实观测时间与到期时间。原 xyz 读者仍能使用该消息。新旧点不能因一包新地图共同“续期”。
旧 xyz 点云无体素时间时，仍按整帧源时间检查新鲜度，不假定存在历史体素的独立寿命。

规划器按 `T_base(control)<-odom * T_odom<-base(source)` 每周期从源快照重新变换，之后才
做范围裁剪与可配置自身过滤。普通规划、急停和后退使用同一组补偿后的点。后向裁剪范围
至少覆盖配置的最大后退距离及机身与后退安全边界，避免原来 -0.75 m 裁剪漏掉 -0.82 m 后缘。
这不提供盲区内未观测障碍的探测能力。

- 源年龄用 ROS 时钟计算，断流看门狗/耗时用 steady clock；两者同时约束。
- 超过 50 ms 的未来时间戳、零时间戳、过期输入拒绝；小量未来误差被视为年龄 0。
- 重复或乱序源帧不刷新接收时刻，不计入急停/恢复连续新帧数。
- `max_pose_extrapolation_sec=0.05`：当前位姿允许有限外推，必须有两帧可用位姿。
- `max_pose_interpolation_gap_sec=0.20`：不能跨越更大的 odom 间隙插值或估计外推速度。
- 位姿大跳变清除旧障碍；ROS 时间回退清除全部输入快照、位姿缓存及相关确认状态。
  单独一个乱序 odom 包会被忽略，不被当作时钟重置。
- 源/控制位姿不可得时状态为 `OBSTACLE_ALIGNMENT_FAILED` 并停车；计算中输入过期则为
  `INPUT_EXPIRED_DURING_PLANNING`，不会把过期规划速度下发。
- 感知失败不会发布伪造空图；合法空观测仍是有效输入，但不能绕过地图历史占用。

路径及控制障碍使用控制开始时刻。命令消息使用发布时刻；`command_latency_ms` 统计从回调
开始到真实命令发布的耗时。`cycle_work_ms` 含路径发布但不含随后本条遥测自身发布。
`previous_cycle_total_ms` 是上一次**完整回调**（包括遥测/状态事件发布）的耗时，
必须与 `previous_control_stamp_ns` 配对；首条没有上一周期数据，统计时排除。
`control_interval_ms` 记录真实调度间隔。`cycle_sequence` 缺口可检测遥测缺失，不能当作所有
相机/录制丢包的计数。正常/急停/阻塞/输入错误分支都产生逐周期记录。

注意：UWB 适配器目前仍重新赋接收时间；本阶段不改变人的位置/速度估计。
源时间对齐仅补偿机器人自身运动，不预测行人的运动。相机/odom/ROS 时钟必须使用一致时间基准。

## 固定场景及验收

每组保留相同参数、机身尺寸、走速与场地，至少重复左右各三个回合，记录失败回合而非只挑成功：

1. 静止墙体：机器人静止、直行、左右转弯，比较 odom 下墙体位置与重复边缘；机身系应随运动正确变化。
2. 延迟注入：点云延迟 0.1/0.3 s、超过超时、重复源帧、断流、零/未来时间戳、odom 间断/跳变、时钟回退。
3. 合法空观测与感知失败分别测试；不得把错误状态判成自由空间。
4. 匀速、慢走、突然停止：记录距离误差、实际启停和停止响应；本阶段不要求解决比例跟随的稳态误差。
5. 左右直角、突然横穿、侧面盲区障碍和后退后缘障碍：检查制动、净空及原有碰撞保护没有退化。
6. 在目标设备上同时运行双目/地图/录制，覆盖典型点数与 5000 点的压力场景，检查完整回调
   P95/P99/最大值与 >50 ms 次数；节点单独的微基准不能作为 20 Hz 实机通过证据。

墙面误差应结合 5 cm 网格、深度与里程计噪声确定；制动延迟和净空门槛由实测底盘响应确定。
当前离散足迹碰撞检查并不提供严格连续扫掠证明，薄障碍/墙角场景若失败需单独修复，不能靠
修改记录指标宣称通过。

## 测试与回放

```bash
colcon test --packages-select go2_uwb_local_follow
colcon test-result --verbose
```

ROS 集成测试使用独立 ROS domain、localhost、模拟时间和 `/test/cmd`，不会连接真实底盘。
测试覆盖平移/转弯补偿、源时间过期、未来时间戳恢复、重复帧急停、体素寿命、过期 odom、
坐标重置、空/坏点云和地图元数据。

回放时只回放所选上游输入，不能把历史 `/cmd_vel`、旧输出地图或旧诊断与新节点的同名输出
混在一起。使用独立 ROS domain，节点 `use_sim_time=true`、`enable_motion=false`，再用
`ros2 bag play ... --clock --topics ...` 选择对应输入。当前控制定时器是 wall timer，改变回放
倍速会改变接收看门狗与控制采样关系；使用 1 倍速，暂停/跳转后的停车与重置属于预期行为。
回放用于比较算法决策，真实距离/制动/净空仍须闭环实机验收。

## 开发环境验证记录

在本次开发机 x86_64、RelWithDebInfo、隔离 ROS 节点与合成输入条件下：

- 录制/分析冒烟测试成功生成 57 个周期的报告，实际参数快照可读取，遥测序号无缺口。
- 5000 点、最多 81 个候选的合成障碍场景，原碰撞循环的规划耗时约 258–268 ms，
  新增发布前时效检查会拒绝过期结果并停车。
- 等价计算优化后，同一场景完整回调均值约 28.4 ms、P99 约 30.1 ms、最大约 30.2 ms；
  新增坐标补偿均值约 0.13 ms。200 点场景完整回调均值约 1.9 ms。
- 对照测试逐点调用原几何距离函数，验证优化后碰撞结果、首个碰撞位姿和最小净空一致。

以上是特定合成场景的开发机结果，不覆盖全部速度档/候选数量，也没有叠加实机双目负载；
不能据此宣布机器人上的 20 Hz 周期或固定墙稳定性已经验收通过。
