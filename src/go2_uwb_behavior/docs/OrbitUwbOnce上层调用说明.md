# OrbitUwbOnce 上层调用说明

## 1. 功能与接口

上层调用该 Action 后，机器人先根据 UWB 距离靠近目标；距离进入目标半径附近后，机器人沿 UWB 目标周围约 1 米的圆周运动一周，完成后停车并返回结果。接近和环绕阶段的速度意图都会经过现有局部速度规划器避障。

| 项目 | 内容 |
| --- | --- |
| Action 名称 | `/go2/orbit_uwb_once` |
| Action 类型 | `go2_uwb_behavior/action/OrbitUwbOnce` |
| 默认半径 | 1.0 米 |
| 默认方向 | 逆时针 |
| 默认超时 | 60 秒 |

该 Action 由统一行为控制器提供。需启动 `behavior_follow_roam.launch.py`，并确保 UWB、`/leg_odom2`、双目障碍感知和局部规划器正常运行。`enable_motion:=false` 时接口和反馈仍可用，但底盘不会实际移动。

## 2. Goal 字段

| 字段 | 类型 | 取值规则 |
| --- | --- | --- |
| `timeout_sec` | `float64` | `0` 使用配置默认值 60 秒；显式值允许 5～180 秒 |
| `orbit_radius` | `float64` | `0` 使用配置默认值 1 米；显式值允许 0.5～2.0 米 |
| `direction` | `int8` | `0` 使用默认逆时针；`1` 逆时针；`-1` 顺时针 |

接近阶段在 UWB 距离进入“目标半径 + 0.05 米”范围后开始环绕。半径、接近容差和超时默认值可在 `behavior_controller.yaml` 中配置。

## 3. 命令行调用

先构建并加载工作空间（若本机尚未构建新接口）：

```bash
cd /home/bird/robot_ws_leg_velocity_lite3/go2_follow_ws
colcon build --symlink-install --packages-up-to go2_uwb_behavior
source install_lite3/setup.bash
```

启动统一行为链路：

```bash
ros2 launch go2_uwb_behavior behavior_follow_roam.launch.py \
  enable_motion:=true cmd_vel_topic:=/cmd_vel
```

调用一米半径、默认逆时针环绕：

```bash
ros2 action send_goal /go2/orbit_uwb_once \
  go2_uwb_behavior/action/OrbitUwbOnce \
  "{timeout_sec: 60.0, orbit_radius: 1.0, direction: 0}" --feedback
```

顺时针调用示例：

```bash
ros2 action send_goal /go2/orbit_uwb_once \
  go2_uwb_behavior/action/OrbitUwbOnce \
  "{timeout_sec: 60.0, orbit_radius: 1.0, direction: -1}" --feedback
```

## 4. Feedback 与完成判断

Feedback 约每 0.2 秒更新一次：

| 字段 | 含义 |
| --- | --- |
| `state` | `STARTING`、`APPROACHING`、`ORBITING`、`INPUT_PAUSED` 或 `STOPPING` |
| `distance` | 当前 UWB 距离，米 |
| `orbit_angle` | 当前累计环绕角度，弧度；完成一周时约为 `6.283` |
| `elapsed_sec` | Action 已运行时间，秒 |

只有 Action 终态为 `STATUS_SUCCEEDED` 且 `result.code == SUCCESS`，才表示完成一周。Result 字段如下：

| 字段 | 含义 |
| --- | --- |
| `code` | 业务结果码 |
| `message` | 结果说明 |
| `elapsed_sec` | 总执行时间，秒 |
| `final_distance` | 结束时机器人到滤波 UWB 目标的距离，米 |
| `completed_angle` | 本次累计环绕角度，弧度 |

常见非成功结果：

| 结果码 | 含义 |
| --- | --- |
| `CANCELED` | 上层取消了任务 |
| `PREEMPTED_BY_MODE` | 被 `IDLE` 或 `STOP` 模式请求抢占 |
| `NOT_READY` | 启动就绪窗口内 UWB、里程计、障碍或规划器未就绪 |
| `BLOCKED` | 局部规划器持续受阻 |
| `INPUT_TIMEOUT` | 关键输入中断且未在恢复窗口内恢复，或 Nav2 启动 |
| `TIMEOUT` | 超过 Goal 指定的总时限 |
| `STOP_UNCONFIRMED` | 已尝试停车，但里程计未能确认停稳 |

上层不要仅凭 Feedback 到达一周就执行后续动作；应等待最终 Result 并同时检查 Action 状态和 `code`。

## 5. Python 调用示例

下面示例发送 Goal、打印反馈并等待最终结果。需要继续支持取消时，应在业务层保存返回的 Goal Handle，再调用其 `cancel_goal_async()`。

```python
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from go2_uwb_behavior.action import OrbitUwbOnce


class OrbitClient(Node):
    """提供一次 UWB 环绕 Action 的同步调用示例。"""

    def __init__(self):
        """初始化 Action 客户端。"""
        super().__init__("orbit_uwb_once_client")
        self._client = ActionClient(
            self, OrbitUwbOnce, "/go2/orbit_uwb_once"
        )

    def _on_feedback(self, message):
        """打印服务端报告的环绕阶段和进度。"""
        feedback = message.feedback
        self.get_logger().info(
            f"state={feedback.state}, distance={feedback.distance:.2f} m, "
            f"angle={feedback.orbit_angle:.2f} rad"
        )

    def run_once(self):
        """发送一米半径的逆时针 Goal 并等待终态。"""
        if not self._client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("未发现 /go2/orbit_uwb_once Action 服务端")

        goal = OrbitUwbOnce.Goal()
        goal.timeout_sec = 60.0
        goal.orbit_radius = 1.0
        goal.direction = 0

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("环绕 Goal 被拒绝：可能有其他任务正在运行")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        result = wrapped_result.result
        success = (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.code == OrbitUwbOnce.Result.SUCCESS
        )
        return success, result


def main():
    """运行示例客户端并输出最终业务结果。"""
    rclpy.init()
    node = OrbitClient()
    try:
        success, result = node.run_once()
        node.get_logger().info(
            f"success={success}, code={result.code}, message={result.message}, "
            f"angle={result.completed_angle:.2f} rad"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

上层取消时，先使用保存的 `goal_handle`：

```python
cancel_future = goal_handle.cancel_goal_async()
rclpy.spin_until_future_complete(node, cancel_future)
```

取消是异步请求；上层仍应等待 `goal_handle.get_result_async()` 返回 `CANCELED` 结果，确认控制器完成停车流程。

## 6. 互斥与异常处理

- 跟随、环绕和漫游 Action 全局互斥。若有任务正在运行，新 Goal 会被拒绝；上层应先取消当前 Goal 并等待其终态。
- `STOP` 模式会锁定底盘并拒绝新任务；需要解除时调用 `/go2/set_behavior`，请求 `mode: 0`（`IDLE`）。
- 遇到 `BLOCKED`、`INPUT_TIMEOUT`、`TIMEOUT` 或 `STOP_UNCONFIRMED` 时，不要触发依赖环绕成功的后续动作；先处理对应输入或停车状态。
