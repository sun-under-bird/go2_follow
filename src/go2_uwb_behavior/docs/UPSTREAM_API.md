# RK/Lite3 UWB 行为上层调用接口

## 1. 接口选择

语音识别层不要直接发布 `/cmd_vel`，只调用以下接口：

| 语音意图 | 接口 | 生命周期 |
| --- | --- | --- |
| “跟着我” | `/go2/follow_uwb`，`FollowUwb` Action | 持续到 cancel 或超时 |
| “原地玩耍” | `/go2/random_roam`，`RandomRoam` Action | 随机走到一个目标后自动结束 |
| “停下” | `/go2/set_behavior`，`SetBehavior` Service，`mode=STOP` | 锁存停车并拒绝新任务 |
| “解除停止” | `/go2/set_behavior`，`mode=IDLE` | 回到可接收任务的空闲状态 |

两个 Action 全局互斥。Goal 被接受时开启双目点云和避障计算；结束、失败或取消后关闭。上层应保存 Action Goal Handle，“停止跟随”通过 cancel 当前 Follow Goal 实现。

## 2. 持续跟随 Action

```text
Action 名：/go2/follow_uwb
类型：go2_uwb_behavior/action/FollowUwb
```

Goal：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `timeout_sec` | `float64` | `0` 表示持续到 cancel；显式值允许 5～3600 秒 |

```bash
ros2 action send_goal /go2/follow_uwb \
  go2_uwb_behavior/action/FollowUwb \
  "{timeout_sec: 0.0}" --feedback
```

Feedback：

| 字段 | 说明 |
| --- | --- |
| `state` | `STARTING`、`FOLLOWING`、`HOLDING`、`INPUT_PAUSED` 或 `STOPPING` |
| `distance` | 当前 UWB 距离，米 |
| `heading` | 当前 UWB 方位误差，弧度 |
| `elapsed_sec` | 已执行时间，秒 |

Result 包含 `code`、`message`、`elapsed_sec` 和 `final_distance`。常见结果码为
`CANCELED`、`NOT_READY`、`TIMEOUT`、`INPUT_TIMEOUT`、`PREEMPTED_BY_MODE` 和
`STOP_UNCONFIRMED`。跟随是持续任务，正常的“停止跟随”应得到 Action
`STATUS_CANCELED` 且业务码为 `CANCELED`。

## 3. 单次随机玩耍 Action

```text
Action 名：/go2/random_roam
类型：go2_uwb_behavior/action/RandomRoam
```

Goal：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `random_seed` | `uint32` | `0` 使用随机种子；非零便于复现 |
| `timeout_sec` | `float64` | `0` 使用默认 30 秒；显式值允许 5～120 秒 |
| `min_radius` | `float64` | 目标到调用时 UWB 原点的最小半径；与最大半径同时为 0 时使用默认值 |
| `max_radius` | `float64` | 目标到调用时 UWB 原点的最大半径；服务端上限默认 2.0 米 |

```bash
ros2 action send_goal /go2/random_roam \
  go2_uwb_behavior/action/RandomRoam \
  "{random_seed: 0, timeout_sec: 30.0, min_radius: 0.5, max_radius: 2.0}" \
  --feedback
```

行为节点优先在 Goal 接收时冻结经过滤波的 UWB `odom` 坐标；若当时样本不足，则在输入首次就绪时冻结。目标按面积均匀分布在圆环内，并通过障碍净空检查。UWB 后续移动不会拖动本次圆心或目标。

Result：

| 字段 | 说明 |
| --- | --- |
| `code`、`message` | 业务结果及说明 |
| `reached_pose` | 任务结束时机器人 `odom` 位姿 |
| `center_pose` | 本次冻结的 UWB 圆心 |
| `target_pose` | 本次随机目标 |
| `owner_distance` | 机器人到冻结圆心的距离 |
| `min_clearance` | 足迹边缘到最近缓存障碍的净空 |

成功必须同时满足：

```text
Action 状态 == STATUS_SUCCEEDED
且 result.code == SUCCESS
```

其他业务码包括 `CANCELED`、`PREEMPTED_BY_MODE`、`NOT_READY`、
`NO_VALID_GOAL`、`BLOCKED`、`TIMEOUT`、`INPUT_TIMEOUT`、`GEOFENCE_STOP` 和
`STOP_UNCONFIRMED`。任何非成功结果都不应继续执行厂家跳跃、摆腿等动作。

## 4. 停车 Service

```text
服务名：/go2/set_behavior
类型：go2_uwb_behavior/srv/SetBehavior
```

只接受：

- `IDLE=0`：解除 STOP，保持空闲零速。
- `STOP=2`：抢占活动 Action，停车后进入锁存状态。

`FOLLOW=1` 和 `ROAM=3` 会被拒绝，因为长任务必须使用可取消、可反馈的 Action。

## 5. 推荐语音层时序

```text
“跟着我”
  -> 发送 FollowUwb Goal
  -> 保存 Goal Handle
  -> 等待 STARTING 变为 FOLLOWING/HOLDING

“别跟了”
  -> cancel 保存的 Follow Goal
  -> 等待 CANCELED Result（节点已停车并关闭点云计算）

“原地玩耍”
  -> 发送 RandomRoam Goal（带 min/max_radius）
  -> 等待 Result
  -> 仅 SUCCEEDED + SUCCESS 时，才允许进入后续厂家动作

“停下”
  -> SetBehavior(STOP)
  -> 当前 Action 被抢占并停车，后续 Goal 被拒绝
```

## 6. 数据与安全约束

机器人侧必须提供：

| 数据 | RK 默认话题 |
| --- | --- |
| UWB 原始数据 | `/libAoa_robot_publisher` |
| 适配后目标点 | `/uwb/target_point` |
| Lite3 里程计 | `/leg_odom2`，`nav_msgs/Odometry` |
| 左右校正图像 | `/camera/camera/infra1/image_rect_raw`、`infra2/image_rect_raw` |
| 相机内参与 TF | CameraInfo、相机坐标系到 `base_footprint` |

运动期间必须持续进行双目感知，不能只在 Goal 开始时计算一帧点云。输入短时中断会立即输出零速并等待默认 3 秒；恢复后可继续，持续失效返回 `INPUT_TIMEOUT`。漫游目标上限默认 2.0 米，仍有 5.6 米限制区和 6 米硬围栏。

新启动链路输出 `/cmd_vel`，不能与旧 `local_follow.launch.py` 或其他速度控制节点同时运行。
