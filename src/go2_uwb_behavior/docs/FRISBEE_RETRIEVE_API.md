# 多 UWB 飞盘衔回导航接口

本仓库负责选中标签的接近、避障、停车和结果返回。拾取动作及整体游戏流程由外部行为树负责。
ID1 默认是人员，ID2 默认是飞盘；编号均可配置，多个 ID 的定位已由用户在基站上验证。
本次基于 `origin/develop` 的 `1c18eec883e94417c01abfa3c7b4ef216ac5d869`，
工作分支为 `feature/uwb-frisbee-retrieve`，不修改厂家滤波算法和静态库。

## 1. 数据链路和兼容边界

```text
同一串口 -> 厂家融合算法（保持原样）-> LibAoaRobotMsg（各 fob_id 独立限频）
                                         |
                            uwb_target_adapter_node（同一安装外参）
                              |                        |
           /uwb/target_point（默认仅 ID1）       /uwb/targets（带 ID）
                              |                        |
               原 Follow/Orbit/RandomRoam       ApproachUwb 的独立缓存
                              \                        /
                         同一 computeControlledTarget 控制核心
                                        |
                         同一 nominal_cmd -> 同一 MPPI
                                        |
                          同一行为节点安全门控 -> /cmd_vel
```

`/uwb/targets` 类型为 `uwb_aoa_pkg/msg/UwbTarget`：

```text
std_msgs/Header header
uint32 target_id
geometry_msgs/Point position
uint8 confidence
bool valid
int8 state
```

`header.stamp` 保留驱动采集时间，`header.frame_id` 为配置的 `target_frame`
（默认 `base_footprint`）。坐标沿用厂家的 x/y，仅做安装平移和偏航变换，
不新增滤波。z 为 0；距离定义为机器人参考坐标系中 `hypot(x, y)`，
并非机身前沿或拾取机构到飞盘的净空。

各 ID 的源时间戳、接收时间、有效性、更新频率和到达历史独立。
`valid=false` 的新帧也发布以立即撤销运动许可；断流不重发旧位置，订阅方自行检查两种年龄。
`/uwb/target_point` 仍是 `PointStamped`，默认只发布有效 ID1，
ID2 不进入原人员缓存或漫游主人滤波器。其旧接口和控制参数保持不变。

适配器参数：

| 参数 | 初始值 | 意义 |
|---|---|---|
| raw_topic | /libAoa_robot_publisher | 厂家消息输入 |
| target_topic | /uwb/target_point | 兼容人员目标 |
| targets_topic | /uwb/targets | 多标签输出 |
| person_target_id | 1 | 兼容话题所选人员 ID |
| valid_target_ids | 未配置 | 代码默认空白名单，接受全部 ID；可设 [1, 2, 3]，须包含人员 ID |
| sensor_offset_x / sensor_offset_y | 0.0 / 0.0 | 基站安装位置，米 |
| sensor_yaw | 0.0 | 安装偏航，弧度 |
| target_timeout_sec | 0.50 | 源时间及接收时间有效窗口，秒 |
| minimum_confidence | 0 | 可选原厂家可信度下限，0 不额外限制 |
| diagnostic_frequency | 2.0 | 无输入时诊断仍持续运行，Hz |

默认最多保存 64 个不同 ID，避免未知编号耗尽内存；不硬编码只有两个标签。
Humble 的参数 YAML 不接受显式 `valid_target_ids: []`（会解析成未设置类型）；
接受全部 ID 时省略该参数，需要白名单时配置非空整数列表。
`/uwb/target_adapter_diagnostics` 输出每 ID 的采集时间、年龄、有效性、
接收更新频率、状态和可信度。源时间相同的不同 ID 都可接受，同 ID 的重复或旧帧拒绝。

## 2. ApproachUwb 完整定义

Action 名默认为 `/go2/approach_uwb`，
类型为 `go2_uwb_behavior/action/ApproachUwb`：

```text
# 指定标签的一次性接近；0 超时使用服务端默认值。
uint32 target_id
float64 stop_distance_m
float64 timeout_sec
---
uint8 SUCCESS=0
uint8 CANCELED=1
uint8 PREEMPTED=2
uint8 NOT_READY=3
uint8 TARGET_LOST=4
uint8 BLOCKED=5
uint8 TIMEOUT=6
uint8 UNSAFE_DISTANCE=7
uint8 STOP_UNCONFIRMED=8
uint8 INPUT_TIMEOUT=9
uint8 code
string message
uint32 target_id
float64 final_distance
float64 elapsed_sec
---
uint8 STARTING=0
uint8 APPROACHING=1
uint8 ARRIVAL_VERIFY=2
uint8 INPUT_PAUSED=3
uint8 STOPPING=4
uint8 state
uint32 target_id
float64 current_distance
float64 heading_error
float64 elapsed_sec
```

`timeout_sec=0` 使用服务端默认 60 秒；非零必须为有限正数且不大于默认上限 180 秒。
总时间包括启动、输入恢复、制动和受阻等待。超时后仍需执行有界停车确认，
因此拿到 Result 最迟还可能增加 `stop_confirmation_timeout_sec`。

`stop_distance_m` 每个 Goal 独立，默认可请求 0.50～5.00 米。
非法范围、NaN、纯目标模式、STOP 锁存或已有运动 Action 都拒绝 Goal
（客户端 `accepted=false`）。ROS 的 Goal 拒绝没有自定义 Result；
`UNSAFE_DISTANCE` 是已接受任务中实际测得间距低于安全下限的结果。

没有新鲜最终目标时 `final_distance` 为 NaN，不能把旧读数用于交接判断。
反馈中的距离、朝向失效时也为 NaN。反馈状态编号依次为：
STARTING=0、APPROACHING=1、ARRIVAL_VERIFY=2、INPUT_PAUSED=3、STOPPING=4。

| 业务码 | 含义 | 上层处理 |
|---|---|---|
| SUCCESS=0 | 指定 ID 距离稳定、最终距离有效且停稳 | 还须 ROS 状态 SUCCEEDED，才进入拾取或下一导航 |
| CANCELED=1 | 主动取消且停车已确认 | 按取消分支处理 |
| PREEMPTED=2 | 被 IDLE/STOP 服务抢占且停车已确认 | STOP 时保持锁存，禁止自动返回 |
| NOT_READY=3 | 启动窗口内指定标签/里程计/点云/规划器未就绪 | 检查输入后显式重试 |
| TARGET_LOST=4 | 指定 ID 持续断流或无效，其他运动输入正常 | 禁止拾取，恢复标签后显式重试 |
| BLOCKED=5 | MPPI 持续阻断或底盘持续没有进展 | 重新检查障碍/目标，不能禁用碰撞保护 |
| TIMEOUT=6 | 总执行时限已过 | 走失败分支 |
| UNSAFE_DISTANCE=7 | 实测间距低于安全下限 | 检查安装外参和停靠参数 |
| STOP_UNCONFIRMED=8 | 无法证明停稳，无论原因为成功、取消或其他故障 | STOP 锁存，禁止交接；人工/上层确认后解除 |
| INPUT_TIMEOUT=9 | 里程计、点云或规划器持续失效，或里程计坐标跳变 | 检查相应输入，不当成标签丢失 |

成功必须同时满足：

```python
response.status == GoalStatus.STATUS_SUCCEEDED and response.result.code == ApproachUwb.Result.SUCCESS
```

取消期间停车失败时，ROS 终态可以为 CANCELED，但业务码为 STOP_UNCONFIRMED。
检查 ROS 状态一个条件是不够的。STOP 抢占后再 cancel，业务原因保留 PREEMPTED，
ROS 终态可以为 CANCELED，STOP 仍保持锁存。

## 3. 到达、安全与停车

正常接近复用原跟随参数副本，将本 Goal 的停靠距离作为期望距离；
最终接近只降低速度上限及提前请求零速度，不引入新的 PID 或 MPPI。
任意非零名义线速度仍不低于底盘最小有效迈步速度。

到达需要指定 ID 的新采集样本连续在
`minimum_approach_distance_m <= distance <= stop_distance_m + arrival_tolerance_m`
范围内，持续 `arrival_stable_sec`。
重复读取同一帧不能计时，输入失效、目标切换、采集间隔过大都重新计时。
到达验证同时请求零线速度和零角速度；随后检查新鲜里程计并再次检查最终距离。
目标在停车时离开容差，先完成停车，再恢复接近或输入恢复流程，不能报告旧位置成功。

新任务、恢复任务都撤销旧转向锁存、线制动、到达和进度历史，
丢弃旧规划命令，并先等现有规划器对零名义速度输出新零命令再放行。
内部规划速度沿用旧 `Twist` 接口，不带源时间戳；依赖同一规划器按顺序处理和发布，
不能用其他发布者向内部话题注入速度。

短时目标或输入失效立即发零；恢复窗口内允许复检后恢复。
过期超过恢复窗口时，优先区分非 UWB 输入故障与仅当前 ID 丢失。
进度检查同时认可实际平移/转向或相对距离减小，人员移动并不要求相对距离每帧下降。
持续 MPPI BLOCKED 或没有进展时失败。总超时始终有效。

停车复用 `updateStopped()` 和 `PoseStationarityMonitor`，
新任务只累计停车后的不同采集时间戳。默认速度和位姿证据必须同时通过。
原 Action 保留已有速度/位姿任选其一的停车兼容规则。

RK/Lite3 的历史配置记录了静止时 `/leg_odom2` twist 约 0.097 m/s 的偏置。
因此新任务的默认严格判定可能报告 STOP_UNCONFIRMED。
先独立确认真实机器人已静止、检查里程计数据源健康，再标定
`approach_allow_pose_only_stop=true`。此时使用连续新采集位姿证明停稳。
**仅依赖同一里程计，软件无法区分真实静止与“冻结位姿但重新打新时间戳”的故障；
需要数据源健康或独立停车证据。** 本次不扩大原 Action 停车判定的改动范围。

## 4. 参数及初始配置

配置文件为 `config/behavior_controller.yaml`。
以下都是待实机标定的初始值，不是安全验收结论：

| 参数 | 初始值 | 意义 |
|---|---|---|
| targets_topic | /uwb/targets | 新目标输入 |
| approach_action_name | /go2/approach_uwb | 新 Action 名 |
| default_approach_timeout_sec / maximum_approach_timeout_sec | 60 / 180 | 默认及最大总时间，秒 |
| minimum_approach_distance_m / maximum_approach_distance_m | 0.50 / 5.00 | Goal 范围及实测最小间距，米 |
| arrival_tolerance_m | 0.10 | 到达距离容差，米 |
| arrival_stable_sec | 0.50 | 新标签帧连续达标时间，秒 |
| approach_slowdown_distance_m | 1.00 | 剩余距离进入保守区，米 |
| approach_final_max_linear_speed | 0.25 | 保守区上限，m/s，至少等于 min_linear_speed |
| approach_response_delay_sec | 0.20 | 提前停车的底盘延迟估计，秒 |
| approach_braking_decel | 0.70 | 提前停车的减速度估计，m/s² |
| approach_require_heading | false | 是否要求最终对准 |
| approach_heading_tolerance_rad | 0.20 | 可选最终朝向容差，弧度 |
| approach_allow_pose_only_stop | false | 是否接受经标定的位姿独立停稳证据 |

复用参数：`target_timeout_sec=0.50`、`odom_timeout_sec=0.20`、
`obstacle_timeout_sec=0.70`、`planner_cmd_timeout_sec=0.20`、
`readiness_timeout_sec=2.0`、`input_recovery_timeout_sec=3.0`、
`stop_confirm_sec=0.30`、`stop_confirmation_timeout_sec=2.0`、
`stop_linear_threshold=0.04`、`stop_angular_threshold=0.08`、
`stop_position_epsilon=0.02`、`stop_yaw_epsilon=0.03`、
`required_progress=0.15`、`progress_window_sec=3.0`、
`planner_blocked_timeout_sec=1.0`。

服务器校验最小接近距离至少覆盖 `robot_clearance_radius`（默认 0.40 米）；
实机配置必须进一步包含安装外参、机构伸出和安全余量。
现有 MPPI 矩形足迹为 0.68×0.38 米，加 0.02 米安全余量，
正前方紧急区约从基座 x=0.36 到 x=0.51 米。
因此 0.50 米 UWB 距离与碰撞/紧急区可能相互影响，具体取决于目标方位、外参和可见点云。
近地面飞盘还可能低于障碍点高度阈值。不能为了接近飞盘关闭障碍保护；
确实无安全运动时应返回 BLOCKED。

## 5. 行为树时序和底盘交接

```text
ApproachUwb(ID2, 0.50 m, 60 s)
    -> 仅 SUCCEEDED + SUCCESS
    -> 将运动所有权交给独立拾取执行器
    -> 拾取成功并完成执行器停车/释放所有权
    -> ApproachUwb(ID1, 上层指定距离, 上层指定超时)
    -> 仅 SUCCEEDED + SUCCESS
    -> 如需要，另启动原 FollowUwb 持续跟随
```

拾取失败时停止流程，不请求 ID1。行为节点不订阅拾取结果，也不自动切换目标。
本节点四个运动 Action 全局互斥，包括停车等待阶段。
跨 Nav2/拾取/其他节点的底盘仲裁仍由上层已有仲裁器负责；
ROS 话题不能自动授予全系统唯一控制权。

配置 `publish_idle_velocity=false` 让空闲阶段释放速度话题。
它不是跨节点运动互斥器；下一执行器必须在上一任务停车结果、零命令和所有权释放后启动。
同一 Action Server 的旧任务仍活跃时，新 Goal 会拒绝，而不会暗中抢占。

取消流程：发送 Action cancel -> 立即撤销非零输出 -> 等待最终 Result ->
确认业务码和停车证据 -> 再考虑下一动作。
仅收到 cancel 接收响应不能当成停稳。
断开客户端连接本身不等同于 cancel。

紧急请求：

```bash
ros2 service call /go2/set_behavior go2_uwb_behavior/srv/SetBehavior "{mode: 2}"
```

STOP 在到达验证和最终停车中也有效，不能被后来的取消/IDLE 请求自动解除。
任务结束后停止自动返回；确认环境和底盘后显式解除：

```bash
ros2 service call /go2/set_behavior go2_uwb_behavior/srv/SetBehavior "{mode: 0}"
```

若仍有运动任务正在停车，IDLE 不能解除已锁存的 STOP；需等待 Result。
STOP 服务只控制本节点，还应使用机器人已有硬件急停及全局仲裁机制。

## 6. Humble 启动和接口命令

RK3588/ARM64：

```bash
cd /home/cat/robot_ws/go2_follow_develop
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to go2_uwb_behavior
source install/setup.bash
# 先检查多标签和状态机，最终速度为零。
ros2 launch go2_uwb_behavior uwb_visual_bringup.launch.py enable_motion:=false
```

沿用既有 RealSense D435i 相机、双目图像/TF 和 `/leg_odom2` 部署；
合并启动文件不另外启动相机或里程计驱动。
先确认串口设备、采集时钟、安装外参、每 ID 的频率及新旧话题：

```bash
ros2 topic echo /uwb/targets
ros2 topic echo /uwb/target_point
ros2 topic echo /uwb/target_adapter_diagnostics
ros2 topic info /cmd_vel --verbose
```

不同时启动旧 `local_follow.launch.py` 与统一行为链。
已有基站开启时可只启动
`behavior_follow_roam.launch.py enable_motion:=false targets_topic:=/uwb/targets`。

前往 ID2：

```bash
ros2 action send_goal /go2/approach_uwb go2_uwb_behavior/action/ApproachUwb \
  "{target_id: 2, stop_distance_m: 0.5, timeout_sec: 60.0}" --feedback
```

拾取成功且执行器已释放运动所有权后，单次返回 ID1：

```bash
ros2 action send_goal /go2/approach_uwb go2_uwb_behavior/action/ApproachUwb \
  "{target_id: 1, stop_distance_m: 1.0, timeout_sec: 60.0}" --feedback
```

上面 send_goal 命令用于独立接口验收。实际行为树按完整 Result 检查再发下一 Goal。
示例客户端只模拟拾取结果，不驱动拾取机构：

```bash
ros2 run go2_uwb_behavior frisbee_retrieve_client.py --simulate-pickup failure
ros2 run go2_uwb_behavior frisbee_retrieve_client.py --simulate-pickup success \
  --frisbee-distance 0.5 --person-distance 1.0 --timeout-sec 60
```

停止用户启动的联调：先在 Action 客户端取消并等待结果，或发 STOP 并等待结果；
再对该 launch 终端按 Ctrl+C。不要全局停止 WSL、Docker 或其他任务。
控制节点正常退出前尽力发零；断电/进程被强杀仍依赖底盘原有命令看门狗。

## 7. WSL 软件测试

本机源码工作树：

```bash
cd "/mnt/c/Users/chy/Documents/ChatGPT/go2_slam 2/go2_follow_develop_20261009"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to go2_uwb_behavior \
  --parallel-workers 2 --cmake-args -DBUILD_UWB_VENDOR_DRIVER=OFF -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
python3 scripts/run_frisbee_tests.py --phase all
```

x86 的 OFF 仅跳过 ARM64 厂家驱动链接，保留消息、串口核心及假融合库驱动测试。
真实厂家静态库必须在 ARM64 编译和实测，不以假库测试声称厂家的多标签融合已被软件验证。

测试脚本用 ROS 域 177 和 localhost；本机 WSL FastDDS 出现“节点发现成功但样本不交付”，
脚本在 CycloneDDS 已安装且未指定 RMW 时只对测试选择它，不改变机器人的中间件。
本机 CycloneDDS 默认较大报文还出现诊断消息丢失；测试脚本默认加载
`scripts/cyclonedds_test.xml` 限制报文大小并分片，现有 `CYCLONEDDS_URI` 优先。
可设置 `FRISBEE_TEST_DOMAIN_ID`、`RMW_IMPLEMENTATION` 或 `CYCLONEDDS_URI` 覆盖测试环境。
集成测试使用 `scripts/ros_test_clock.py` 持续推进隔离的采集时钟，
避开本机 WSL 系统时间回退；断流、恢复窗口、任务超时和停车超时仍使用真实经过时间。
部署 launch 不启用这个测试时钟，也不加载测试 DDS 配置。
脚本建立独立进程组，正常、失败、超时和 Ctrl+C 均只回收此次测试启动的进程并确认退出。

分阶段命令：

```bash
python3 scripts/run_frisbee_tests.py --phase data
python3 scripts/run_frisbee_tests.py --phase approach
colcon test-result --test-result-base build --verbose
```

新增测试包括：每 ID 数据/时钟、旧话题隔离、串口背靠背多 ID 和拔插、
新样本到达计时、安全距离、可选朝向、切换后重置，
真实 ROS Action 的 A～G 场景、非 UWB 故障、超时、受阻、取消停车失败，
示例客户端 H（拾取失败不得启动 ID1），以及真实 MPPI 的软件闭环接近和碰撞阻断。
全量测试继续运行原 Follow/RandomRoam、Orbit 互斥及取消、导航目标生成、
滚动障碍、动态转向制动、MPPI 和紧急恢复测试。

实际构建、测试报告和修改清单见 [软件验证与交付记录](FRISBEE_VALIDATION.md)。

## 8. 实机验收边界

用户已验证实际基站多 ID 和坐标正确；本次没有进行机器人运动验收。
软件闭环只用平面速度积分，不包含真实制动延迟、脚步动力学或拾取器动作。

先 enable_motion=false 验证接口，再在空旷场地用固定标签代替飞盘：
记录源/接收年龄、频率、ID、变换后位置、配置/实测距离、实测 twist、
位姿变化、名义/规划/最终速度及停车后距离。
标定最小间距、最终速度、提前制动估计和停稳证据后才测试真实飞盘。
分别验证 ID1 移动返回、标签失联、取消、STOP、障碍 BLOCKED，
并检查三种旧 Action 的原实机行为。验收由用户按记录执行，本次不代为宣称通过。
