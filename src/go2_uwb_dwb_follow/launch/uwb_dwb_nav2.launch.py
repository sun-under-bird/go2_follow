from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """启动 DWB 跟随链，并让 controller_server 直接输出最终 /cmd_vel."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    follow_params_file = LaunchConfiguration("follow_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

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
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
                remappings=[("cmd_vel", "/cmd_vel")],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
                remappings=[("cmd_vel", "/cmd_vel")],
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
                    {"node_names": ["controller_server", "behavior_server"]},
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
            Node(
                package="behavior_ext_plugins",
                executable="follow_path_recovery_bt_node",
                name="dwb_follow_recovery_bt",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "path_topic": "/uwb_dwb/path",
                        "status_topic": "/follow/dwb_recovery_status",
                        "controller_id": "FollowPath",
                        "goal_checker_id": "general_goal_checker",
                        "recovery_retries": 2,
                        "recovery_distance_m": 0.30,
                        "recovery_speed_mps": 0.12,
                    }
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
        ]
    )
