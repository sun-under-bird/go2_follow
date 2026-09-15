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

"""启动不改动原节点的统一 UWB 跟随、随机漫游与双目避障链路."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# 组装新行为节点及原有感知、滚动地图和局部规划器。
def generate_launch_description() -> LaunchDescription:
    behavior_share = Path(get_package_share_directory("go2_uwb_behavior"))
    follow_share = Path(get_package_share_directory("go2_uwb_local_follow"))

    behavior_params_file = LaunchConfiguration("behavior_params_file")
    follow_params_file = LaunchConfiguration("follow_params_file")
    stereo_params_file = LaunchConfiguration("stereo_params_file")
    planner_params_file = LaunchConfiguration("planner_params_file")
    rolling_map_params_file = LaunchConfiguration("rolling_map_params_file")
    raw_uwb_topic = LaunchConfiguration("raw_uwb_topic")
    target_topic = LaunchConfiguration("target_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    base_frame = LaunchConfiguration("base_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    obstacle_topic = LaunchConfiguration("obstacle_topic")
    depth_observation_topic = LaunchConfiguration("depth_observation_topic")
    rolling_obstacle_topic = LaunchConfiguration("rolling_obstacle_topic")
    nominal_cmd_topic = LaunchConfiguration("nominal_cmd_topic")
    planner_cmd_topic = LaunchConfiguration("planner_cmd_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    enable_motion = LaunchConfiguration("enable_motion")
    enable_heading_relaxation = LaunchConfiguration("enable_avoidance_heading_relaxation")

    stereo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(follow_share / "launch" / "stereo_obstacle_cloud.launch.py")
        ),
        launch_arguments={
            "params_file": stereo_params_file,
            "obstacle_cloud_topic": obstacle_topic,
            "ray_observation_topic": depth_observation_topic,
            "base_frame": base_frame,
        }.items(),
    )

    adapter_node = Node(
        package="go2_uwb_local_follow",
        executable="uwb_target_adapter_node",
        name="uwb_target_adapter_node",
        output="screen",
        parameters=[
            follow_params_file,
            {
                "raw_topic": raw_uwb_topic,
                "target_topic": target_topic,
                "target_frame": base_frame,
            },
        ],
    )

    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(follow_share / "launch" / "local_velocity_planner.launch.py")
        ),
        launch_arguments={
            "params_file": planner_params_file,
            "rolling_map_params_file": rolling_map_params_file,
            "nominal_cmd_topic": nominal_cmd_topic,
            "observation_topic": depth_observation_topic,
            "rolling_obstacle_topic": rolling_obstacle_topic,
            "odom_topic": odom_topic,
            "base_frame": base_frame,
            "odom_frame": odom_frame,
            # 规划器只向内部话题发布，行为节点负责最终模式和围栏门控。
            "cmd_vel_topic": planner_cmd_topic,
            "enable_motion": enable_motion,
        }.items(),
    )

    behavior_node = Node(
        package="go2_uwb_behavior",
        executable="uwb_behavior_controller_node",
        name="uwb_behavior_controller_node",
        output="screen",
        parameters=[
            behavior_params_file,
            {
                "base_frame": base_frame,
                "odom_frame": odom_frame,
                "target_topic": target_topic,
                "odom_topic": odom_topic,
                "obstacle_topic": rolling_obstacle_topic,
                "nominal_cmd_topic": nominal_cmd_topic,
                "planner_cmd_topic": planner_cmd_topic,
                "enable_avoidance_heading_relaxation": ParameterValue(
                    enable_heading_relaxation, value_type=bool
                ),
                "cmd_vel_topic": cmd_vel_topic,
                "enable_motion": ParameterValue(enable_motion, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "behavior_params_file",
                default_value=str(behavior_share / "config" / "behavior_controller.yaml"),
            ),
            DeclareLaunchArgument(
                "follow_params_file",
                default_value=str(follow_share / "config" / "uwb_follow_only.yaml"),
            ),
            DeclareLaunchArgument(
                "stereo_params_file",
                default_value=str(follow_share / "config" / "stereo_obstacle_cloud.yaml"),
            ),
            DeclareLaunchArgument(
                "planner_params_file",
                default_value=str(follow_share / "config" / "local_velocity_planner.yaml"),
            ),
            DeclareLaunchArgument(
                "rolling_map_params_file",
                default_value=str(follow_share / "config" / "rolling_obstacle_map.yaml"),
            ),
            DeclareLaunchArgument("raw_uwb_topic", default_value="/libAoa_robot_publisher"),
            DeclareLaunchArgument("target_topic", default_value="/uwb/target_point"),
            DeclareLaunchArgument("odom_topic", default_value="/odom_leg"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("obstacle_topic", default_value="/local_grid_obstacle"),
            DeclareLaunchArgument(
                "depth_observation_topic", default_value="/local_depth_observation"
            ),
            DeclareLaunchArgument(
                "rolling_obstacle_topic", default_value="/local_rolling_obstacle"
            ),
            DeclareLaunchArgument(
                "nominal_cmd_topic", default_value="/go2_uwb_local_follow/nominal_cmd"
            ),
            DeclareLaunchArgument(
                "planner_cmd_topic", default_value="/go2_uwb_behavior/planner_cmd_vel"
            ),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("enable_motion", default_value="true"),
            DeclareLaunchArgument("enable_avoidance_heading_relaxation", default_value="true"),
            stereo_launch,
            adapter_node,
            planner_launch,
            behavior_node,
        ]
    )
