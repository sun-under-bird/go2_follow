import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """启动目标过滤、局部规划、控制适配和唯一的安全速度出口."""
    pkg_share = get_package_share_directory("go2_dynamic_follow_avoidance")
    default_config = os.path.join(pkg_share, "config", "go2_dynamic_follow_avoidance.yaml")

    config_file = LaunchConfiguration("config_file")
    use_simple_follow = LaunchConfiguration("use_simple_follow")
    one1000_topic = LaunchConfiguration("one1000_topic")
    one1000_msg_type = LaunchConfiguration("one1000_msg_type")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    disable_quality_gating = LaunchConfiguration("disable_quality_gating")
    bypass_safety = LaunchConfiguration("bypass_safety")
    require_odom_watchdog = LaunchConfiguration("require_odom_watchdog")
    require_pointcloud_watchdog = LaunchConfiguration("require_pointcloud_watchdog")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("use_simple_follow", default_value="false"),
            DeclareLaunchArgument("one1000_topic", default_value="/uwb/target_point"),
            DeclareLaunchArgument("one1000_msg_type", default_value="geometry_msgs/msg/PointStamped"),
            DeclareLaunchArgument("pointcloud_topic", default_value="/local_grid_obstacle"),
            DeclareLaunchArgument("disable_quality_gating", default_value="false"),
            DeclareLaunchArgument("bypass_safety", default_value="false"),
            DeclareLaunchArgument("require_odom_watchdog", default_value="true"),
            DeclareLaunchArgument("require_pointcloud_watchdog", default_value="true"),
            Node(
                package="go2_dynamic_follow_avoidance",
                executable="follow_goal_node",
                name="follow_goal_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "one1000_topic": one1000_topic,
                        "one1000_msg_type": one1000_msg_type,
                        "disable_quality_gating": disable_quality_gating,
                    },
                ],
            ),
            Node(
                package="go2_dynamic_follow_avoidance",
                executable="local_path_planner",
                name="local_path_planner",
                output="screen",
                condition=UnlessCondition(use_simple_follow),
                parameters=[
                    config_file,
                    {
                        "pointcloud_topic": pointcloud_topic,
                    },
                ],
            ),
            Node(
                package="go2_dynamic_follow_avoidance",
                executable="follow_path_action_client",
                name="follow_path_action_client",
                output="screen",
                condition=UnlessCondition(use_simple_follow),
                parameters=[config_file],
            ),
            Node(
                package="go2_dynamic_follow_avoidance",
                executable="simple_follow_controller",
                name="simple_follow_controller",
                output="screen",
                condition=IfCondition(use_simple_follow),
                parameters=[config_file],
            ),
            Node(
                package="go2_dynamic_follow_avoidance",
                executable="safety_mux",
                name="safety_mux",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "pointcloud_topic": pointcloud_topic,
                        "cmd_vel_in": "/cmd_vel_nav",
                        "cmd_vel_out": "/cmd_vel",
                        "bypass_safety": bypass_safety,
                        "require_odom_watchdog": require_odom_watchdog,
                        "require_pointcloud_watchdog": require_pointcloud_watchdog,
                    },
                ],
            ),
        ]
    )
