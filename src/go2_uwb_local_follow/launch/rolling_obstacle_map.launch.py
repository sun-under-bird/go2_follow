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

"""独立启动使用 /odom_leg 运动补偿的滚动局部障碍地图."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# 组装滚动地图节点并开放输入、输出、里程计和坐标系参数。
def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("go2_uwb_local_follow"))
    default_params = str(package_share / "config" / "rolling_obstacle_map.yaml")

    params_file = LaunchConfiguration("params_file")
    input_observation_topic = LaunchConfiguration("input_observation_topic")
    output_obstacle_topic = LaunchConfiguration("output_obstacle_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    base_frame = LaunchConfiguration("base_frame")
    odom_frame = LaunchConfiguration("odom_frame")

    rolling_map_node = Node(
        package="go2_uwb_local_follow",
        executable="rolling_obstacle_map_node",
        name="rolling_obstacle_map_node",
        output="screen",
        parameters=[
            params_file,
            {
                "base_frame": base_frame,
                "odom_child_frame": base_frame,
                "odom_frame": odom_frame,
                "input_observation_topic": input_observation_topic,
                "output_obstacle_topic": output_obstacle_topic,
                "odom_topic": odom_topic,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument(
                "input_observation_topic", default_value="/local_depth_observation"
            ),
            DeclareLaunchArgument(
                "output_obstacle_topic", default_value="/local_rolling_obstacle"
            ),
            DeclareLaunchArgument("odom_topic", default_value="/odom_leg"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            rolling_map_node,
        ]
    )
