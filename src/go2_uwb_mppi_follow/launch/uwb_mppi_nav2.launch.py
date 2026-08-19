from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """启动 MPPI 跟随与空闲区域恢复链，所有速度直接输出 /cmd_vel."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    start_uwb = LaunchConfiguration("start_uwb")
    uwb_device = LaunchConfiguration("uwb_device")
    uwb_frame_id = LaunchConfiguration("uwb_frame_id")
    uwb_publish_rate_hz = LaunchConfiguration("uwb_publish_rate_hz")
    uwb_params_file = LaunchConfiguration("uwb_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
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
                "start_uwb",
                default_value="true",
                description="Start the UWB serial driver and target-point converter.",
            ),
            DeclareLaunchArgument(
                "uwb_device",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-FTDI_FT232R_USB_UART_AG00S82A-if00-port0"
                ),
                description="Stable serial path of the external UWB receiver.",
            ),
            DeclareLaunchArgument(
                "uwb_frame_id",
                default_value="base_footprint",
                description="Frame assigned to aligned external UWB measurements.",
            ),
            DeclareLaunchArgument(
                "uwb_publish_rate_hz",
                default_value="10.0",
                description="Maximum UWB message publish rate.",
            ),
            DeclareLaunchArgument(
                "uwb_params_file",
                default_value=default_uwb_params_file,
                description="Parameter file for the UWB target and direct MPPI follow nodes.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params_file,
                description="Parameter file for Nav2 controller, costmap, and recovery behavior.",
            ),
            Node(
                package="uwb_aoa_pkg",
                executable="libAoa_robot_example",
                name="libAoa_robot_publisher",
                output="screen",
                arguments=[uwb_device],
                parameters=[
                    {
                        "frame_id": uwb_frame_id,
                        "publish_rate_hz": ParameterValue(
                            uwb_publish_rate_hz,
                            value_type=float,
                        ),
                    }
                ],
                condition=IfCondition(start_uwb),
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
                    {
                        "node_names": [
                            "controller_server",
                            "behavior_server",
                        ]
                    },
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="uwb_aoa_pkg",
                executable="one1000_target_point_node",
                name="one1000_target_point_node",
                output="screen",
                parameters=[
                    uwb_params_file,
                    {"use_sim_time": use_sim_time},
                ],
                arguments=["--ros-args", "--log-level", "warn"],
                condition=IfCondition(start_uwb),
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
                    {"cmd_vel_topic": "/cmd_vel"},
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="behavior_ext_plugins",
                executable="follow_path_recovery_bt_node",
                name="mppi_follow_recovery_bt",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "path_topic": "/uwb_follow/path",
                        "status_topic": "/follow/mppi_recovery_status",
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
