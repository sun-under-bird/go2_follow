import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("go2_dynamic_follow_avoidance")
    default_follow_config = os.path.join(pkg_share, "config", "go2_dynamic_follow_avoidance.yaml")
    default_nav2_config = os.path.join(pkg_share, "config", "nav2_mppi_controller.yaml")

    start_go2_driver = LaunchConfiguration("start_go2_driver")
    start_twist_bridge = LaunchConfiguration("start_twist_bridge")
    start_camera = LaunchConfiguration("start_camera")
    start_rtabmap = LaunchConfiguration("start_rtabmap")
    start_nav2_controller = LaunchConfiguration("start_nav2_controller")
    use_simple_follow = LaunchConfiguration("use_simple_follow")
    start_uwb = LaunchConfiguration("start_uwb")
    start_follow = LaunchConfiguration("start_follow")
    use_sim_time = LaunchConfiguration("use_sim_time")
    follow_config = LaunchConfiguration("follow_config")
    nav2_params = LaunchConfiguration("nav2_params")
    disable_quality_gating = LaunchConfiguration("disable_quality_gating")
    bypass_safety = LaunchConfiguration("bypass_safety")
    require_odom_watchdog = LaunchConfiguration("require_odom_watchdog")
    require_pointcloud_watchdog = LaunchConfiguration("require_pointcloud_watchdog")
    uwb_device = LaunchConfiguration("uwb_device")
    uwb_frame_id = LaunchConfiguration("uwb_frame_id")
    uwb_publish_rate_hz = LaunchConfiguration("uwb_publish_rate_hz")

    camera_launch = PathJoinSubstitution(
        [FindPackageShare("stereo_camera_pkg"), "launch", "usb_400.launch.py"]
    )
    rtabmap_launch = os.path.join(pkg_share, "launch", "d435i_rtabmap.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_go2_driver", default_value="false"),
            DeclareLaunchArgument("start_twist_bridge", default_value="false"),
            DeclareLaunchArgument("start_camera", default_value="false"),
            DeclareLaunchArgument("start_rtabmap", default_value="false"),
            DeclareLaunchArgument("start_nav2_controller", default_value="true"),
            DeclareLaunchArgument("use_simple_follow", default_value="false"),
            DeclareLaunchArgument("start_uwb", default_value="false"),
            DeclareLaunchArgument("start_follow", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("follow_config", default_value=default_follow_config),
            DeclareLaunchArgument("nav2_params", default_value=default_nav2_config),
            DeclareLaunchArgument("disable_quality_gating", default_value="true"),
            DeclareLaunchArgument("bypass_safety", default_value="true"),
            DeclareLaunchArgument("require_odom_watchdog", default_value="true"),
            DeclareLaunchArgument("require_pointcloud_watchdog", default_value="true"),
            DeclareLaunchArgument("uwb_device", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("uwb_frame_id", default_value="uwb_link"),
            DeclareLaunchArgument("uwb_publish_rate_hz", default_value="1.0"),
            Node(
                package="go2_driver",
                executable="driver",
                name="go2_driver",
                output="screen",
                condition=IfCondition(start_go2_driver),
            ),
            Node(
                package="go2_twist_bridge",
                executable="twist_bridge",
                name="go2_twist_bridge",
                output="screen",
                remappings=[("cmd_vel", "/cmd_vel_safe")],
                condition=IfCondition(start_twist_bridge),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch),
                condition=IfCondition(start_camera),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_launch),
                launch_arguments={
                    "base_frame": "base_footprint",
                    "odom_topic": "/odom_leg",
                    "use_viz": "false",
                }.items(),
                condition=IfCondition(start_rtabmap),
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": use_sim_time}],
                remappings=[("cmd_vel", "/cmd_vel_nav")],
                condition=IfCondition(start_nav2_controller),
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_controller",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": [
                            "controller_server",
                        ],
                    }
                ],
                condition=IfCondition(start_nav2_controller),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "dynamic_follow_avoidance.launch.py")
                ),
                launch_arguments={
                    "config_file": follow_config,
                    "use_simple_follow": use_simple_follow,
                    "one1000_topic": "/libAoa_robot_publisher",
                    "one1000_msg_type": "uwb_aoa_pkg/msg/LibAoaRobotMsg",
                    "disable_quality_gating": disable_quality_gating,
                    "pointcloud_topic": "/local_grid_obstacle",
                    "cmd_vel_in": "/cmd_vel_nav",
                    "cmd_vel_out": "/cmd_vel_safe",
                    "bypass_safety": bypass_safety,
                    "require_odom_watchdog": require_odom_watchdog,
                    "require_pointcloud_watchdog": require_pointcloud_watchdog,
                }.items(),
                condition=IfCondition(start_follow),
            ),
        ]
    )
