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

"""启动厂家 UWB 目标适配和不带避障的第一阶段跟随验收链路."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


# 组装 UWB 目标适配和纯跟随控制，不启动厂家驱动或双目避障。
def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("go2_uwb_local_follow"))
    default_params = str(package_share / "config" / "uwb_follow_only.yaml")

    params_file = LaunchConfiguration("params_file")
    raw_topic = LaunchConfiguration("raw_topic")
    target_topic = LaunchConfiguration("target_topic")
    target_frame = LaunchConfiguration("target_frame")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    enable_motion = LaunchConfiguration("enable_motion")

    adapter_node = Node(
        package="go2_uwb_local_follow",
        executable="uwb_target_adapter_node",
        name="uwb_target_adapter_node",
        output="screen",
        parameters=[
            params_file,
            {
                "raw_topic": raw_topic,
                "target_topic": target_topic,
                "target_frame": target_frame,
            },
        ],
    )

    follow_node = Node(
        package="go2_uwb_local_follow",
        executable="uwb_follow_controller_node",
        name="uwb_follow_controller_node",
        output="screen",
        parameters=[
            params_file,
            {
                "target_topic": target_topic,
                "cmd_vel_topic": cmd_vel_topic,
                "enable_motion": ParameterValue(enable_motion, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument(
                "raw_topic", default_value="/libAoa_robot_publisher"
            ),
            DeclareLaunchArgument("target_topic", default_value="/uwb/target_point"),
            DeclareLaunchArgument("target_frame", default_value="base_footprint"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_follow"),
            DeclareLaunchArgument("enable_motion", default_value="true"),
            adapter_node,
            follow_node,
        ]
    )
