from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("cloud_in", default_value="/local_grid_obstacle"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument("uwb_target_topic", default_value="/libAoa_robot_publisher"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel_safe"),
        DeclareLaunchArgument("uwb_input_frame", default_value="uwb_link"),

        # Convert the forward stereo obstacle cloud into a LaserScan for the core controller.
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            remappings=[
                ("cloud_in", LaunchConfiguration("cloud_in")),
                ("scan", LaunchConfiguration("scan_topic")),
            ],
            parameters=[{
                "target_frame": LaunchConfiguration("target_frame"),
                "angle_min": -1.57,
                "angle_max": 1.57,
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.01,
                "range_max": 3.0,
                "min_height": 0.02,
                "max_height": 0.8,
                "use_inf": True,
            }],
        ),

        # Core UWB follow and obstacle avoidance node.
        Node(
            package="jie_deamon",
            executable="robot_nexus",
            name="robot_nexus",
            output="screen",
            parameters=[{
                "active": True,
                "scan_topic": LaunchConfiguration("scan_topic"),
                "uwb_target_topic": LaunchConfiguration("uwb_target_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "target_frame": LaunchConfiguration("target_frame"),
                "uwb_input_frame": LaunchConfiguration("uwb_input_frame"),
                "follow_dist": 1.0,
                "target_timeout_sec": 0.5,
                "scan_timeout_sec": 0.5,
                "target_exclusion_radius": 0.35,
                "apf_influence_dist": 0.8,
                "apf_slowdown_dist": 0.8,
                "apf_emergency_dist": 0.4,
                "max_linear_speed": 0.5,
                "max_lateral_speed": 0.12,
                "max_angular_speed": 1.0,
                "linear_y_scale_factor": 1.0,
                "lateral_deadband": 0.03,
                "rectangle_width": 0.35,
            }],
        ),
    ])
