from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """启动 Smac Hybrid、TEB 与空闲区域恢复，速度直接输出 /cmd_vel."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    uwb_params_file = LaunchConfiguration("uwb_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    default_uwb_params_file = PathJoinSubstitution(
        [
            FindPackageShare("go2_uwb_teb_follow"),
            "config",
            "uwb_teb_follow.yaml",
        ]
    )
    default_nav2_params_file = PathJoinSubstitution(
        [
            FindPackageShare("go2_uwb_teb_follow"),
            "config",
            "nav2_smac_teb.yaml",
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
                description="Parameter file for the UWB TEB follow node.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params_file,
                description="Parameter file for Smac Hybrid, TEB, costmaps, and recovery.",
            ),
            Node(
                package="go2_uwb_mppi_follow",
                executable="target_obstacle_filter_node",
                name="teb_target_obstacle_filter_node",
                output="screen",
                parameters=[
                    {
                        "input_cloud_topic": "/local_grid_obstacle",
                        "output_cloud_topic": "/local_grid_obstacle_filtered",
                        "target_topic": "/uwb_teb/target",
                        "filter_frame": "base_footprint",
                        "clear_radius_m": 0.45,
                        "clear_z_min_m": 0.05,
                        "clear_z_max_m": 2.0,
                        "pass_through_without_target": True,
                        "use_latest_tf": False,
                    }
                ],
                arguments=["--ros-args", "--log-level", "warn"],
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
                name="lifecycle_manager_smac_teb",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": ["planner_server", "controller_server", "behavior_server"]},
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="go2_uwb_teb_follow",
                executable="uwb_teb_follow_node",
                name="uwb_teb_follow_node",
                output="screen",
                parameters=[
                    uwb_params_file,
                    {"use_sim_time": use_sim_time},
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="behavior_ext_plugins",
                executable="follow_path_recovery_bt_node",
                name="teb_follow_recovery_bt",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "path_topic": "/uwb_teb/path",
                        "status_topic": "/follow/teb_recovery_status",
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
