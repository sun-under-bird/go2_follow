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

"""启动使用实测初始速度、制动尾段和足迹碰撞检查的局部速度规划器."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# 组装完整局部速度采样节点，默认只输出隔离的零实机速度。
def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("go2_uwb_local_follow"))
    default_params = str(package_share / "config" / "local_velocity_planner.yaml")

    params_file = LaunchConfiguration("params_file")
    nominal_cmd_topic = LaunchConfiguration("nominal_cmd_topic")
    obstacle_topic = LaunchConfiguration("obstacle_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    enable_motion = LaunchConfiguration("enable_motion")

    planner_node = Node(
        package="go2_uwb_local_follow",
        executable="local_velocity_planner_node",
        name="local_velocity_planner_node",
        output="screen",
        parameters=[
            params_file,
            {
                "nominal_cmd_topic": nominal_cmd_topic,
                "obstacle_topic": obstacle_topic,
                "odom_topic": odom_topic,
                "cmd_vel_topic": cmd_vel_topic,
                "enable_motion": ParameterValue(enable_motion, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument(
                "nominal_cmd_topic",
                default_value="/go2_uwb_local_follow/nominal_cmd",
            ),
            DeclareLaunchArgument(
                "obstacle_topic", default_value="/local_grid_obstacle"
            ),
            DeclareLaunchArgument("odom_topic", default_value="/odom_leg"),
            DeclareLaunchArgument(
                "cmd_vel_topic", default_value="/cmd_vel_planned"
            ),
            DeclareLaunchArgument("enable_motion", default_value="false"),
            planner_node,
        ]
    )
