# Go2 UWB 行为控制

本包在不修改原 `go2_uwb_local_follow` 的前提下，新增统一 FOLLOW、IDLE、STOP
和单次随机漫游控制，并复用原双目滚动地图与局部速度规划器。

给上层应用开发者的完整调用约定见
[上层调用接口文档](docs/UPSTREAM_API.md)。

新链路中的速度所有权固定为：

```text
uwb_behavior_controller_node
  -> /go2_uwb_local_follow/nominal_cmd
  -> local_velocity_planner_node
  -> /go2_uwb_behavior/planner_cmd_vel
  -> uwb_behavior_controller_node 安全门控
  -> /cmd_vel
```

不要同时启动原 `local_follow.launch.py`，否则原跟随节点会与统一行为节点竞争名义速度。

## 启动

先按原方式启动 UWB 串口与相机驱动，再启动新链路：

```bash
ros2 launch go2_uwb_behavior behavior_follow_roam.launch.py \
  enable_motion:=true \
  cmd_vel_topic:=/cmd_vel
```

首次架空检查使用：

```bash
ros2 launch go2_uwb_behavior behavior_follow_roam.launch.py enable_motion:=false
```

`enable_motion:=false` 会保留订阅、选点、反馈、诊断及规划计算，但最终 `/cmd_vel`
始终为零，适合先验证接口和状态机。

## 上层接口

执行一次随机漫游并接收过程反馈：

```bash
ros2 action send_goal /go2/random_roam \
  go2_uwb_behavior/action/RandomRoam \
  "{random_seed: 0, timeout_sec: 30.0}" --feedback
```

恢复持续 UWB 跟随：

```bash
ros2 service call /go2/set_behavior \
  go2_uwb_behavior/srv/SetBehavior "{mode: 1}"
```

切换 IDLE 或锁存 STOP：

```bash
ros2 service call /go2/set_behavior \
  go2_uwb_behavior/srv/SetBehavior "{mode: 0}"
ros2 service call /go2/set_behavior \
  go2_uwb_behavior/srv/SetBehavior "{mode: 2}"
```

一次漫游 Action 成功后节点停在 IDLE，上层只有在收到成功结果并确认动作净空需求后，
才能调用不经过 `/cmd_vel` 的厂家动作接口。执行动作期间不得有其他节点发布
`/cmd_vel`。

## 状态和安全语义

- 默认模式为 FOLLOW；一次 Action 接管控制后进入 ROAM，成功后进入 IDLE。
- STOP 是锁存模式，STOP 状态下拒绝新漫游；必须先显式切换到 IDLE 或 FOLLOW。
- FOLLOW、STOP、取消请求都会让活动中的 Action 先发布零速度并等待里程计停车确认。
- UWB、里程计、滚动障碍点云或规划器速度超时会终止当前漫游。
- 目标受阻时最多重新选点两次；主人超过 5 米时先返回，5.6～6 米区间禁止继续外扩。
- 只有连续 0.30 秒满足线速度小于 0.04 m/s、角速度小于 0.08 rad/s，Action
  才能返回成功。

Action 结果码：

| 结果码 | 含义 |
| --- | --- |
| `SUCCESS` | 到达目标并确认停稳 |
| `CANCELED` | 上层取消 |
| `PREEMPTED_BY_MODE` | 被 FOLLOW/IDLE/STOP 抢占 |
| `NOT_READY` | 启动阶段输入尚未齐备 |
| `NO_VALID_GOAL` | 50 次内没有满足约束的候选目标 |
| `BLOCKED` | 目标或返回路径持续受阻 |
| `TIMEOUT` | 超过本次 Action 总时限 |
| `INPUT_TIMEOUT` | 运动期间关键输入断流 |
| `GEOFENCE_STOP` | 规划速度持续触发 6 米围栏 |
| `STOP_UNCONFIRMED` | 已到达但未能从里程计确认停稳 |

## 验证

```bash
colcon build --symlink-install --packages-up-to go2_uwb_behavior
colcon test --packages-select go2_uwb_local_follow go2_uwb_behavior
colcon test-result --verbose
```

实机应按以下顺序逐级放开：

1. 使用 `enable_motion:=false` 检查 Action、Service、诊断和随机目标。
2. 将 `random_goal_radius_max` 临时改为 `2.5`，保持 `0.35 m/s` 低速，在架空或安全场地测试。
3. 验证取消、STOP、FOLLOW 抢占、传感器断流和障碍物阻断均能停车。
4. 确认 `/cmd_vel` 和 `/go2_uwb_local_follow/nominal_cmd` 各只有一个发布者后，再恢复正式半径参数。
