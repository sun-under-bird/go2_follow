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

"""为集成测试提供跨用例连续的采集时钟，避免 WSL 系统授时回退干扰安全断言."""
import time

from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock


class MonotonicTestClock:
    """只替代采集时间；行为期限和接收时效仍由真实 steady clock 计时."""

    def __init__(self, node, topic):
        """建立独立时钟话题，节点通过 use_sim_time 和 remap 接入."""
        self.publisher = node.create_publisher(Clock, topic, 10)

    def sample(self):
        """从主机单调时钟取采集时间，跨测试节点重建仍不会回退."""
        # 不以用例启动时刻为零点，否则下一个用例会制造时间回退。
        nanoseconds = time.monotonic_ns()
        stamp = Time(sec=nanoseconds // 1000000000, nanosec=nanoseconds % 1000000000)
        self.publisher.publish(Clock(clock=stamp))
        return stamp
