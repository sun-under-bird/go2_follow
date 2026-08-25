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

"""一键启动双目障碍感知、UWB 名义跟随和局部速度规划链路."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# 直接组装全部功能节点，并确保只有局部规划器接管真实速度话题。
def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("go2_uwb_local_follow"))
    config_directory = package_share / "config"

    stereo_params_file = LaunchConfiguration("stereo_params_file")
    follow_params_file = LaunchConfiguration("follow_params_file")
    planner_params_file = LaunchConfiguration("planner_params_file")
    left_image = LaunchConfiguration("left_image")
    left_camera_info = LaunchConfiguration("left_camera_info")
    right_image = LaunchConfiguration("right_image")
    right_camera_info = LaunchConfiguration("right_camera_info")
    disparity_topic = LaunchConfiguration("disparity_topic")
    obstacle_topic = LaunchConfiguration("obstacle_topic")
    raw_uwb_topic = LaunchConfiguration("raw_uwb_topic")
    target_topic = LaunchConfiguration("target_topic")
    base_frame = LaunchConfiguration("base_frame")
    odom_topic = LaunchConfiguration("odom_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    enable_motion = LaunchConfiguration("enable_motion")
    publish_debug_depth = LaunchConfiguration("publish_debug_depth")

    disparity_node = Node(
        package="stereo_image_proc",
        executable="disparity_node",
        name="disparity_node",
        output="screen",
        parameters=[stereo_params_file],
        remappings=[
            ("left/image_rect", left_image),
            ("left/camera_info", left_camera_info),
            ("right/image_rect", right_image),
            ("right/camera_info", right_camera_info),
            ("disparity", disparity_topic),
        ],
    )

    projector_node = Node(
        package="go2_uwb_local_follow",
        executable="stereo_obstacle_projector_node",
        name="stereo_obstacle_projector_node",
        output="screen",
        parameters=[
            stereo_params_file,
            {
                "base_frame": base_frame,
                "disparity_topic": disparity_topic,
                "camera_info_topic": left_camera_info,
                "obstacle_cloud_topic": obstacle_topic,
                "publish_debug_depth": ParameterValue(
                    publish_debug_depth, value_type=bool
                ),
            },
        ],
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

    follow_node = Node(
        package="go2_uwb_local_follow",
        executable="uwb_follow_controller_node",
        name="uwb_follow_controller_node",
        output="screen",
        parameters=[
            follow_params_file,
            {
                "base_frame": base_frame,
                "target_topic": target_topic,
                "cmd_vel_topic": "/cmd_vel_follow",
                # 名义控制只给规划器提供输入，禁止绕过避障直接控制底盘。
                "enable_motion": False,
            },
        ],
    )

    planner_node = Node(
        package="go2_uwb_local_follow",
        executable="local_velocity_planner_node",
        name="local_velocity_planner_node",
        output="screen",
        parameters=[
            planner_params_file,
            {
                "base_frame": base_frame,
                "odom_child_frame": base_frame,
                "nominal_cmd_topic": "/go2_uwb_local_follow/nominal_cmd",
                "obstacle_topic": obstacle_topic,
                "odom_topic": odom_topic,
                "cmd_vel_topic": cmd_vel_topic,
                "enable_motion": ParameterValue(enable_motion, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "stereo_params_file",
                default_value=str(config_directory / "stereo_obstacle_cloud.yaml"),
            ),
            DeclareLaunchArgument(
                "follow_params_file",
                default_value=str(config_directory / "uwb_follow_only.yaml"),
            ),
            DeclareLaunchArgument(
                "planner_params_file",
                default_value=str(config_directory / "local_velocity_planner.yaml"),
            ),
            DeclareLaunchArgument(
                "left_image",
                default_value="/camera/camera/infra1/image_rect_raw",
            ),
            DeclareLaunchArgument(
                "left_camera_info",
                default_value="/camera/camera/infra1/camera_info",
            ),
            DeclareLaunchArgument(
                "right_image",
                default_value="/camera/camera/infra2/image_rect_raw",
            ),
            DeclareLaunchArgument(
                "right_camera_info",
                default_value="/camera/camera/infra2/camera_info",
            ),
            DeclareLaunchArgument(
                "disparity_topic", default_value="/stereo/disparity"
            ),
            DeclareLaunchArgument(
                "obstacle_topic", default_value="/local_grid_obstacle"
            ),
            DeclareLaunchArgument(
                "raw_uwb_topic", default_value="/libAoa_robot_publisher"
            ),
            DeclareLaunchArgument("target_topic", default_value="/uwb/target_point"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("odom_topic", default_value="/odom_leg"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("enable_motion", default_value="true"),
            DeclareLaunchArgument("publish_debug_depth", default_value="false"),
            disparity_node,
            projector_node,
            adapter_node,
            follow_node,
            planner_node,
        ]
    )
