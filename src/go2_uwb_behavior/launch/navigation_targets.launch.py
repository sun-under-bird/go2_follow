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

"""只生成 UWB 随机导航目标，不创建底盘速度发布者."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """复用统一启动文件并固定纯目标与禁运动模式."""
    share = Path(get_package_share_directory("go2_uwb_behavior"))
    return LaunchDescription([
        # 纯目标行为节点保留原服务，不启动新的运动控制链。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / "launch" / "behavior_follow_roam.launch.py")),
            launch_arguments={"target_only": "true", "enable_motion": "false"}.items(),
        ),
    ])
