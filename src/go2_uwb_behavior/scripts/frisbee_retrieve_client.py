#!/usr/bin/env python3
# Copyright 2026 OpenAI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""行为树时序示例：接近 ID2、模拟拾取结果、成功后才接近 ID1."""
import argparse

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient

from go2_uwb_behavior.action import ApproachUwb


def is_success(response):
    """只有 ROS SUCCEEDED 和业务 SUCCESS 同时成立才允许进入下一动作."""
    return response.status == GoalStatus.STATUS_SUCCEEDED and (
        response.result.code == ApproachUwb.Result.SUCCESS)


def run_retrieve_sequence(approach, pickup, frisbee_distance=0.5, person_distance=1.0):
    """把两个导航和拾取握手串联；拾取失败立即结束，不启动人员返回."""
    if not is_success(approach(2, frisbee_distance)):
        return False
    if not pickup():
        return False
    return is_success(approach(1, person_distance))


class RetrieveClient:
    """阻塞示例客户端，保留当前 Goal 句柄以便 Ctrl+C 时取消并等待停车结果."""

    def __init__(self, node, action_name, timeout_sec):
        """初始化 Action 客户端，不创建底盘速度发布者."""
        self.node = node
        self.client = ActionClient(node, ApproachUwb, action_name)
        self.timeout_sec = timeout_sec
        self.active_handle = None

    def wait(self, future):
        """仅等待 ROS 响应，任务总超时和停车上限由服务端执行."""
        rclpy.spin_until_future_complete(self.node, future)
        if not future.done():
            raise RuntimeError("ROS 上下文关闭，无法取得任务结果")
        return future.result()

    def approach(self, target_id, distance):
        """请求一次指定 ID 的接近，拒绝请求也不会继续拾取/返回."""
        if not self.client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("ApproachUwb 服务不可用")
        goal = ApproachUwb.Goal(
            target_id=target_id, stop_distance_m=distance, timeout_sec=self.timeout_sec)
        handle = self.wait(self.client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError("Goal 被拒绝，请检查距离范围、STOP 和其他运动任务")
        self.active_handle = handle
        response = self.wait(handle.get_result_async())
        self.active_handle = None
        self.node.get_logger().info(
            f"ID{target_id}: status={response.status}, code={response.result.code}, "
            f"distance={response.result.final_distance:.3f}, {response.result.message}")
        return response

    def cancel_active(self):
        """取消只算请求；拿到停车结果前不能把底盘交给下一执行器."""
        if self.active_handle and rclpy.ok():
            self.wait(self.active_handle.cancel_goal_async())
            response = self.wait(self.active_handle.get_result_async())
            self.node.get_logger().info(
                f"取消结果 code={response.result.code}: {response.result.message}")
            self.active_handle = None


def main():
    """运行显式选择拾取成功/失败的接口演示，不执行真实拾取."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--action-name", default="/go2/approach_uwb")
    parser.add_argument("--frisbee-distance", type=float, default=0.5)
    parser.add_argument("--person-distance", type=float, default=1.0)
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    parser.add_argument("--simulate-pickup", choices=("success", "failure"), required=True)
    args = parser.parse_args()
    # 不让默认 SIGINT 提前关闭 ROS，上层 Ctrl+C 先走 Action cancel 停车握手。
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    node = rclpy.create_node("frisbee_retrieve_example")
    client = RetrieveClient(node, args.action_name, args.timeout_sec)
    try:
        def pickup():
            """模拟外部拾取执行器的独立结果，仅用于验证行为树握手."""
            success = args.simulate_pickup == "success"
            node.get_logger().info(f"模拟拾取结果: {args.simulate_pickup}")
            return success

        success = run_retrieve_sequence(
            client.approach, pickup, args.frisbee_distance, args.person_distance)
        return 0 if success else 1
    except KeyboardInterrupt:
        client.cancel_active()
        return 130
    except RuntimeError as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
