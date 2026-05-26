from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    follow_params_file = LaunchConfiguration("follow_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    cmd_vel_out = LaunchConfiguration("cmd_vel_out")

    default_follow_params_file = PathJoinSubstitution(
        [
            FindPackageShare("go2_uwb_dwb_follow"),
            "config",
            "uwb_dwb_follow.yaml",
        ]
    )
    default_nav2_params_file = PathJoinSubstitution(
        [
            FindPackageShare("go2_uwb_dwb_follow"),
            "config",
            "nav2_dwb_controller.yaml",
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
                "autostart",
                default_value="true",
                description="Automatically configure and activate Nav2 lifecycle nodes.",
            ),
            DeclareLaunchArgument(
                "follow_params_file",
                default_value=default_follow_params_file,
                description="Parameter file for UWB DWB follow helper nodes.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params_file,
                description="Parameter file for Nav2 DWB controller server and local costmap.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_out",
                default_value="/cmd_vel_safe",
                description="Topic used by the Go2 velocity bridge.",
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
                remappings=[("cmd_vel", cmd_vel_out)],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_controller",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": ["controller_server"]},
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="go2_uwb_dwb_follow",
                executable="uwb_point_follow_node",
                name="uwb_point_follow_node",
                output="screen",
                parameters=[follow_params_file, {"use_sim_time": use_sim_time}],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
        ]
    )
