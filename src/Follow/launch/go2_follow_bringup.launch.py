import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """启动统一的 UWB、双目感知、局部规划与安全控制链."""
    pkg_share = get_package_share_directory("go2_dynamic_follow_avoidance")
    default_follow_config = os.path.join(pkg_share, "config", "go2_dynamic_follow_avoidance.yaml")
    default_nav2_config = os.path.join(pkg_share, "config", "nav2_mppi_controller.yaml")

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
    rtabmap_launch = os.path.join(pkg_share, "launch", "d435i_rtabmap.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_rtabmap", default_value="false"),
            DeclareLaunchArgument("start_nav2_controller", default_value="true"),
            DeclareLaunchArgument("use_simple_follow", default_value="false"),
            DeclareLaunchArgument("start_uwb", default_value="false"),
            DeclareLaunchArgument("start_follow", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("follow_config", default_value=default_follow_config),
            DeclareLaunchArgument("nav2_params", default_value=default_nav2_config),
            DeclareLaunchArgument("disable_quality_gating", default_value="false"),
            DeclareLaunchArgument("bypass_safety", default_value="false"),
            DeclareLaunchArgument("require_odom_watchdog", default_value="true"),
            DeclareLaunchArgument("require_pointcloud_watchdog", default_value="true"),
            DeclareLaunchArgument("uwb_device", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("uwb_frame_id", default_value="uwb_link"),
            DeclareLaunchArgument("uwb_publish_rate_hz", default_value="10.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_launch),
                launch_arguments={
                    "base_frame": "base_footprint",
                    "odom_topic": "/odom",
                    "use_viz": "false",
                    "left_image": "/camera/camera/infra1/image_rect_raw",
                    "right_image": "/camera/camera/infra2/image_rect_raw",
                    "left_camera_info": "/camera/camera/infra1/camera_info",
                    "right_camera_info": "/camera/camera/infra2/camera_info",
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
                    "one1000_topic": "/uwb/target_point",
                    "one1000_msg_type": "geometry_msgs/msg/PointStamped",
                    "disable_quality_gating": disable_quality_gating,
                    "pointcloud_topic": "/local_grid_obstacle",
                    "bypass_safety": bypass_safety,
                    "require_odom_watchdog": require_odom_watchdog,
                    "require_pointcloud_watchdog": require_pointcloud_watchdog,
                }.items(),
                condition=IfCondition(start_follow),
            ),
        ]
    )
