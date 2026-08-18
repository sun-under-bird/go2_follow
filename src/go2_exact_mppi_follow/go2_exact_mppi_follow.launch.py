from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create launch description for UWB goal bridge and EXACT-MPPI controller."""

    params_file = LaunchConfiguration("params_file")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    one1000_topic = LaunchConfiguration("one1000_topic")

    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("go2_exact_mppi_follow"),
            "config",
            "go2_exact_mppi_follow.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Parameter file for Go2 EXACT-MPPI following.",
            ),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/local_grid_obstacle",
                description="Stereo PointCloud2 topic.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odom",
                description="Odometry topic.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Final Go2 velocity command topic.",
            ),
            DeclareLaunchArgument(
                "one1000_topic",
                default_value="/libAoa_robot_publisher",
                description="ONE1000/UWB target topic.",
            ),
            Node(
                package="go2_exact_mppi_follow",
                executable="go2_uwb_goal_bridge",
                name="go2_uwb_goal_bridge",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "one1000_topic": one1000_topic,
                    },
                ],
            ),
            Node(
                package="go2_exact_mppi_follow",
                executable="go2_exact_mppi_node",
                name="go2_exact_mppi_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "pointcloud_topic": pointcloud_topic,
                        "odom_topic": odom_topic,
                        "cmd_vel_topic": cmd_vel_topic,
                    },
                ],
            ),
        ]
    )
