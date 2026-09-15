# Go2 UWB 跟随与随机漫游上层调用接口

## 1. 给调用方的结论

上层语音、交互或任务节点只需要调用两个 ROS 2 接口：

| 用途 | 接口 | 类型 |
| --- | --- | --- |
| 切换常驻模式 | `/go2/set_behavior` | `go2_uwb_behavior/srv/SetBehavior` |
| 执行一次随机漫游 | `/go2/random_roam` | `go2_uwb_behavior/action/RandomRoam` |

调用随机漫游时，上层不需要提供 UWB 坐标、随机目标坐标、障碍点云或速度。行为节点会从
UWB、里程计和双目避障链路中自行取得这些数据，选择一个固定目标，经过局部避障规划器移动，
停稳后通过 Action Result 返回结果。

上层必须遵守两条规则：

1. 只有收到 `STATUS_SUCCEEDED` 且结果码为 `SUCCESS` 后，才能执行随机动作。
2. 上层及动作节点不得并行发布 `/cmd_vel`；厂家动作接口也必须与本节点的速度控制互斥。

## 2. 模式切换 Service

### 2.1 接口定义

```text
服务名：/go2/set_behavior
类型：go2_uwb_behavior/srv/SetBehavior
```

请求字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `mode` | `uint8` | 请求切换到的行为模式 |

模式常量：

| 常量 | 数值 | 说明 |
| --- | ---: | --- |
| `IDLE` | 0 | 停止跟随并保持零速度，等待上层任务 |
| `FOLLOW` | 1 | 恢复 UWB 跟随与避障 |
| `STOP` | 2 | 锁存停车；此状态下拒绝新的漫游 Action |
| `ROAM` | 3 | 仅用于状态表示，不能通过 Service 启动 |

响应字段：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `accepted` | `bool` | 本次模式请求是否被接受 |
| `current_mode` | `uint8` | 响应时节点所处模式；抢占停车期间可能仍为 `ROAM` |
| `message` | `string` | 人类可读的处理说明 |

### 2.2 命令行调用

恢复跟随：

```bash
ros2 service call /go2/set_behavior \
  go2_uwb_behavior/srv/SetBehavior "{mode: 1}"
```

进入空闲：

```bash
ros2 service call /go2/set_behavior \
  go2_uwb_behavior/srv/SetBehavior "{mode: 0}"
```

紧急或任务级锁存停车：

```bash
ros2 service call /go2/set_behavior \
  go2_uwb_behavior/srv/SetBehavior "{mode: 2}"
```

如果当前正在漫游，`IDLE`、`FOLLOW` 或 `STOP` 请求会抢占 Action。节点先输出零速度，
等待里程计确认停稳，再以 `PREEMPTED_BY_MODE` 结束 Action，最后进入请求的模式。

## 3. 单次随机漫游 Action

### 3.1 接口定义

```text
Action 名：/go2/random_roam
类型：go2_uwb_behavior/action/RandomRoam
```

Goal 字段：

| 字段 | 类型 | 单位 | 说明 |
| --- | --- | --- | --- |
| `random_seed` | `uint32` | - | `0` 表示每次使用随机种子；非零值便于复现选点 |
| `timeout_sec` | `float64` | s | `0` 使用默认 30 秒；显式值允许 5～120 秒 |

命令行示例：

```bash
ros2 action send_goal /go2/random_roam \
  go2_uwb_behavior/action/RandomRoam \
  "{random_seed: 0, timeout_sec: 30.0}" --feedback
```

### 3.2 Feedback 字段

| 字段 | 类型 | 单位 | 说明 |
| --- | --- | --- | --- |
| `state` | `uint8` | - | 当前漫游阶段 |
| `target_pose` | `geometry_msgs/PoseStamped` | m | 本次固定目标，坐标系默认为 `odom` |
| `distance_remaining` | `float64` | m | 到目标的剩余距离；返回阶段表示到 4.5 米释放边界的距离 |
| `owner_distance` | `float64` | m | 机器人到滤波后主人位置的距离 |
| `elapsed_sec` | `float64` | s | 本次 Action 已运行时间 |

Feedback 状态常量：

| 常量 | 数值 | 说明 |
| --- | ---: | --- |
| `PREPARING` | 0 | 等待输入就绪并确认机器人停稳 |
| `SELECTING_GOAL` | 1 | 在半径、步长和障碍净空约束内选点 |
| `NAVIGATING` | 2 | 经过原局部规划器向固定目标移动 |
| `RETURNING` | 3 | 主人距离超过 5 米，优先返回主人方向 |
| `STOPPING` | 4 | 到达、失败、取消或抢占后的停车确认 |
| `RETRYING` | 5 | 当前目标受阻，停车后准备重新选点 |

### 3.3 Result 字段与成功判定

| 字段 | 类型 | 单位 | 说明 |
| --- | --- | --- | --- |
| `code` | `uint8` | - | 业务结果码 |
| `message` | `string` | - | 结果说明 |
| `reached_pose` | `geometry_msgs/PoseStamped` | m | Action 结束时机器人 `odom` 位姿 |
| `owner_distance` | `float64` | m | Action 结束时主人距离 |
| `min_clearance` | `float64` | m | 机器人足迹边缘到最近缓存障碍点的净空 |

上层成功条件必须同时满足：

```text
ROS Action 状态 == STATUS_SUCCEEDED
并且
result.code == SUCCESS
```

业务结果码：

| 常量 | 数值 | Action 终态 | 上层处理建议 |
| --- | ---: | --- | --- |
| `SUCCESS` | 0 | SUCCEEDED | 可以进入随机动作链路 |
| `CANCELED` | 1 | CANCELED | 不执行动作，保持 IDLE |
| `PREEMPTED_BY_MODE` | 2 | ABORTED | 服从最新模式，不执行动作 |
| `NOT_READY` | 3 | ABORTED | 检查 UWB、里程计、障碍和规划器输入 |
| `NO_VALID_GOAL` | 4 | ABORTED | 当前可活动空间不足，稍后再试 |
| `BLOCKED` | 5 | ABORTED | 路径持续受阻，不执行动作 |
| `TIMEOUT` | 6 | ABORTED | 本次任务超时，不执行动作 |
| `INPUT_TIMEOUT` | 7 | ABORTED | 运动中输入断流，检查传感链路 |
| `GEOFENCE_STOP` | 8 | ABORTED | 速度持续触发 6 米围栏，不执行动作 |
| `STOP_UNCONFIRMED` | 9 | ABORTED | 未确认停稳，严禁执行动作 |

`min_clearance` 在空点云时可能为正无穷，在输入已经失效时可能为 NaN；调用方不应只依赖
这个字段判定能否做动作。需要大范围摆腿、跳跃或翻滚的动作，应由上层使用独立的动作安全
检查，并扩大所需净空。

## 4. 推荐的上层调用时序

“你可以原地玩耍了”：

```text
发送 /go2/random_roam Goal
  -> Goal 被接受
  -> 持续读取 Feedback，可用于 UI 或语音状态
  -> 等待 Result
  -> 仅 SUCCESS：执行一个厂家动作
  -> 动作结束后保持 IDLE，或按产品逻辑再次漫游
```

“跟着我”：

```text
调用 /go2/set_behavior，mode=FOLLOW
  -> 若正在漫游：等待 Action 返回 PREEMPTED_BY_MODE
  -> 行为节点确认停车后进入 FOLLOW
```

“停下”：

```text
调用 /go2/set_behavior，mode=STOP
  -> STOP 锁存
  -> 后续漫游 Goal 会被拒绝
  -> 需要恢复时显式发送 IDLE 或 FOLLOW
```

同一时间只允许存在一个漫游 Goal。上层收到 Goal 被拒绝时，应先查询自己的任务状态，
不要高频重复发送。

## 5. Python 调用示例

下面示例展示接口用法。实际产品代码还应增加连接超时、任务 ID、日志和动作执行互斥锁。

```python
from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from go2_uwb_behavior.action import RandomRoam
from go2_uwb_behavior.srv import SetBehavior


class Go2BehaviorClient(Node):
    """封装上层需要使用的行为 Service 和漫游 Action."""

    def __init__(self):
        """创建行为模式与随机漫游客户端."""
        super().__init__("go2_behavior_client")
        self.mode_client = self.create_client(
            SetBehavior, "/go2/set_behavior"
        )
        self.roam_client = ActionClient(
            self, RandomRoam, "/go2/random_roam"
        )

    def follow(self):
        """请求恢复 UWB 跟随模式."""
        request = SetBehavior.Request()
        request.mode = SetBehavior.Request.FOLLOW
        return self.mode_client.call_async(request)

    def stop(self):
        """请求进入锁存停车模式."""
        request = SetBehavior.Request()
        request.mode = SetBehavior.Request.STOP
        return self.mode_client.call_async(request)

    def roam_once(self):
        """异步发起一次随机漫游并注册结果处理函数."""
        goal = RandomRoam.Goal()
        goal.random_seed = 0
        goal.timeout_sec = 30.0
        future = self.roam_client.send_goal_async(
            goal, feedback_callback=self.on_feedback
        )
        future.add_done_callback(self.on_goal_response)

    def on_feedback(self, feedback_message):
        """接收漫游阶段、剩余距离和主人距离."""
        feedback = feedback_message.feedback
        self.get_logger().info(
            f"state={feedback.state}, remaining="
            f"{feedback.distance_remaining:.2f} m, owner="
            f"{feedback.owner_distance:.2f} m"
        )

    def on_goal_response(self, future):
        """在 Goal 被接受后开始等待最终结果."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("随机漫游请求被拒绝")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        """只在 Action 和业务结果同时成功时通知动作层."""
        wrapped = future.result()
        succeeded = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.code == RandomRoam.Result.SUCCESS
        )
        if succeeded:
            self.get_logger().info("漫游完成且已停稳，可以请求动作层")
            # 在这里通知动作层；动作层仍需执行自身的动作净空检查。
        else:
            self.get_logger().warning(
                f"漫游未成功：code={wrapped.result.code}, "
                f"message={wrapped.result.message}"
            )
```

## 6. 机器人侧必须具备的数据

以下数据由机器人底层、UWB 和避障链路提供，不是上层调用 Goal 的字段：

| 数据 | 默认话题与类型 | 关键要求 |
| --- | --- | --- |
| UWB 原始目标 | `/libAoa_robot_publisher`，`uwb_aoa_pkg/msg/LibAoaRobotMsg` | `x/y` 有限，建议稳定 10 Hz |
| 适配后主人点 | `/uwb/target_point`，`geometry_msgs/msg/PointStamped` | 默认在 `base_footprint`，超时阈值 0.50 s |
| 腿部里程计 | `/odom_leg`，`nav_msgs/msg/Odometry` | `frame_id=odom`、`child_frame_id=base_footprint`，建议至少 20 Hz |
| 左右校正图像 | `/camera/camera/infra1/image_rect_raw`、`infra2/image_rect_raw` | 与各自 CameraInfo 时间同步 |
| 左相机内参 | `/camera/camera/infra1/camera_info` | 相机参数有效 |
| 相机到机身 TF | 相机坐标系到 `base_footprint` | 可按图像时间戳查询 |
| 滚动障碍点云 | `/local_rolling_obstacle`，`sensor_msgs/msg/PointCloud2` | 由本项目链路生成，超时阈值 0.70 s |
| 规划器内部速度 | `/go2_uwb_behavior/planner_cmd_vel`，`geometry_msgs/msg/Twist` | 由原局部规划器生成，超时阈值 0.20 s |

行为节点对主人位置使用 5 帧中值、低通滤波、0.4 米中心死区和 1 秒持续确认。
随机目标距离主人 1.5～4.5 米、单步 0.8～1.8 米，目标障碍净空至少 0.7 米。

## 7. 启动与联调检查

新启动文件不会启动相机驱动和机器人 `/odom_leg` 发布节点。先启动这些硬件数据源，再启动：

```bash
source install/setup.bash
ros2 launch uwb_aoa_pkg uwb_source.launch.py serial_port:=/dev/ttyUSB0
```

```bash
source install/setup.bash
ros2 launch go2_uwb_behavior behavior_follow_roam.launch.py \
  enable_motion:=false
```

第一次联调必须使用 `enable_motion:=false`。此时可以验证服务、Action 接收、目标选择、
Feedback 和诊断，但机器人不会真正到达目标，Action 最终出现受阻或超时是正常现象。

建议检查：

```bash
ros2 topic hz /libAoa_robot_publisher
ros2 topic hz /odom_leg
ros2 topic hz /local_rolling_obstacle
ros2 topic echo /go2/behavior_diagnostics --once
ros2 topic info /cmd_vel --verbose
ros2 topic info /go2_uwb_local_follow/nominal_cmd --verbose
```

最后两项应各自只有一个发布者。使用新启动链时，不要同时启动原
`go2_uwb_local_follow/local_follow.launch.py`。

接口验证完成后，先把 `random_goal_radius_max` 改为 `2.5` 米，在封闭安全场地低速验证，
确认到达、取消、STOP、FOLLOW 抢占、输入断流和障碍阻断都能停车，再恢复正式参数。

## 输入短暂失效时的兼容语义

输入超时后 Action 先暂停，诊断为 `ROAM_INPUT_PAUSED`，Feedback 仍使用原有
`STOPPING` 数值，不新增接口字段。默认恢复窗口为 3 秒，可通过
`input_recovery_timeout_sec` 配置；窗口内全部输入恢复并确认停稳后继续原目标。
超过恢复窗口才返回 `INPUT_TIMEOUT`，Action 总超时不会因为暂停而延长。取消和
模式切换仍可随时抢占；显式 STOP 不会被数据恢复自动解除。
