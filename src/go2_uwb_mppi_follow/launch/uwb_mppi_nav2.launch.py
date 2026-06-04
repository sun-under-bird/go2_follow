from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    uwb_params_file = LaunchConfiguration("uwb_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    cmd_vel_nav = LaunchConfiguration("cmd_vel_nav")
    cmd_vel_out = LaunchConfiguration("cmd_vel_out")

    default_uwb_params_file = PathJoinSubstitution(
        [
            FindPackageShare("go2_uwb_mppi_follow"),
            "config",
            "uwb_mppi_follow.yaml",
        ]
    )
    default_nav2_params_file = PathJoinSubstitution(
        [
            FindPackageShare("go2_uwb_mppi_follow"),
            "config",
            "nav2_mppi_controller.yaml",
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
                "uwb_params_file",
                default_value=default_uwb_params_file,
                description="Parameter file for the UWB target and planner client nodes.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params_file,
                description="Parameter file for Nav2 planner, controller, costmaps, and velocity smoother.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_nav",
                default_value="/cmd_vel_nav",
                description="Raw velocity topic from Nav2 controller_server.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_out",
                default_value="/cmd_vel_safe",
                description="Smoothed velocity topic used by the Go2 velocity bridge.",
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
                remappings=[("cmd_vel", cmd_vel_nav)],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            # 启动 Nav2 恢复行为服务器，用于加载 BackUpTwzFree 脱困插件。
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
                remappings=[("cmd_vel", cmd_vel_nav)],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("cmd_vel", cmd_vel_nav),
                    ("cmd_vel_smoothed", cmd_vel_out),
                ],
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
                    {
                        "node_names": [
                            "planner_server",
                            "controller_server",
                            "behavior_server",
                            "velocity_smoother",
                        ]
                    },
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="go2_uwb_mppi_follow",
                executable="one1000_target_point_node",
                name="one1000_target_point_node",
                output="screen",
                parameters=[
                    uwb_params_file,
                    {"use_sim_time": use_sim_time},
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="go2_uwb_mppi_follow",
                executable="target_obstacle_filter_node",
                name="target_obstacle_filter_node",
                output="screen",
                parameters=[
                    uwb_params_file,
                    {"use_sim_time": use_sim_time},
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="go2_uwb_mppi_follow",
                executable="uwb_follow_path_node",
                name="uwb_follow_path_node",
                output="screen",
                parameters=[
                    uwb_params_file,
                    {"use_sim_time": use_sim_time},
                    {"stop_cmd_vel_topic": cmd_vel_nav},
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
        ]
    )
