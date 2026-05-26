from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("go2_uwb_dwb_follow"),
            "config",
            "uwb_dwb_follow.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Parameter file for UWB DWB follow helper nodes.",
            ),
            Node(
                package="go2_uwb_dwb_follow",
                executable="uwb_point_follow_node",
                name="uwb_point_follow_node",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
        ]
    )
