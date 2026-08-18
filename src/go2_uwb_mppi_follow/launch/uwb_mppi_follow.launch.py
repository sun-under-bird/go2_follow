from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("go2_uwb_mppi_follow"),
            "config",
            "uwb_mppi_follow.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="UWB direct MPPI follow parameter file.",
            ),
            Node(
                package="uwb_aoa_pkg",
                executable="one1000_target_point_node",
                name="one1000_target_point_node",
                output="screen",
                parameters=[params_file],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="go2_uwb_mppi_follow",
                executable="uwb_follow_path_node",
                name="uwb_follow_path_node",
                output="screen",
                parameters=[params_file],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
        ]
    )
