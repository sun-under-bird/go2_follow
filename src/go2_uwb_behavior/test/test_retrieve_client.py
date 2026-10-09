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

"""不连接机器人，验证示例客户端的拾取结果与双重成功握手."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

from action_msgs.msg import GoalStatus
from go2_uwb_behavior.action import ApproachUwb

path = Path(__file__).resolve().parents[1] / "scripts" / "frisbee_retrieve_client.py"
spec = importlib.util.spec_from_file_location("retrieve_client", path)
client = importlib.util.module_from_spec(spec)
spec.loader.exec_module(client)


class TestRetrieveSequence(unittest.TestCase):
    """确保示例与行为树推荐顺序一致."""

    def response(self, status=GoalStatus.STATUS_SUCCEEDED, code=ApproachUwb.Result.SUCCESS):
        """构造 Action 的最终传输状态和业务码."""
        return SimpleNamespace(status=status, result=SimpleNamespace(code=code))

    def test_pickup_failure_never_starts_person_return(self):
        """H：拾取失败只请求 ID2."""
        calls = []

        def approach(tag, distance):
            """记录请求并返回模拟接近成功."""
            calls.append((tag, distance))
            return self.response()

        self.assertFalse(client.run_retrieve_sequence(approach, lambda: False))
        self.assertEqual(calls, [(2, 0.5)])

    def test_success_sequence_and_both_success_requirements(self):
        """正常请求顺序为 ID2、拾取、ID1；任一失败不能交接."""
        calls = []

        def approach(tag, distance):
            """记录正确的动态距离和 ID."""
            calls.append((tag, distance))
            return self.response()

        self.assertTrue(client.run_retrieve_sequence(approach, lambda: True, 0.6, 1.2))
        self.assertEqual(calls, [(2, 0.6), (1, 1.2)])
        for response in (
                self.response(GoalStatus.STATUS_ABORTED),
                self.response(code=ApproachUwb.Result.STOP_UNCONFIRMED)):
            picked = []
            self.assertFalse(client.run_retrieve_sequence(
                lambda tag, distance: response, lambda: picked.append(True)))
            self.assertEqual(picked, [])


if __name__ == "__main__":
    unittest.main()
