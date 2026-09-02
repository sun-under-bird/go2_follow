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

"""启动已校正双目图像的视差计算与机身坐标障碍点云投影."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


# 组装独立双目障碍感知链路，不启动相机驱动或机器人控制器。
def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("go2_uwb_local_follow"))
    default_params = str(package_share / "config" / "stereo_obstacle_cloud.yaml")

    params_file = LaunchConfiguration("params_file")
    left_image = LaunchConfiguration("left_image")
    left_camera_info = LaunchConfiguration("left_camera_info")
    right_image = LaunchConfiguration("right_image")
    right_camera_info = LaunchConfiguration("right_camera_info")
    disparity_topic = LaunchConfiguration("disparity_topic")
    obstacle_cloud_topic = LaunchConfiguration("obstacle_cloud_topic")
    ray_observation_topic = LaunchConfiguration("ray_observation_topic")
    base_frame = LaunchConfiguration("base_frame")
    publish_debug_depth = LaunchConfiguration("publish_debug_depth")

    disparity_node = Node(
        package="stereo_image_proc",
        executable="disparity_node",
        name="disparity_node",
        output="screen",
        parameters=[params_file],
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
            params_file,
            {
                "base_frame": base_frame,
                "disparity_topic": disparity_topic,
                "camera_info_topic": left_camera_info,
                "obstacle_cloud_topic": obstacle_cloud_topic,
                "ray_observation_topic": ray_observation_topic,
                "publish_debug_depth": ParameterValue(
                    publish_debug_depth, value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
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
            DeclareLaunchArgument("disparity_topic", default_value="/stereo/disparity"),
            DeclareLaunchArgument(
                "obstacle_cloud_topic", default_value="/local_grid_obstacle"
            ),
            DeclareLaunchArgument(
                "ray_observation_topic", default_value="/local_depth_observation"
            ),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("publish_debug_depth", default_value="false"),
            disparity_node,
            projector_node,
        ]
    )
