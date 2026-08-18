from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """启动消费 D435i 局部障碍点云并直接输出 /cmd_vel 的跟随控制器."""

    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")

    common_speed_limits = {
        "max_linear": 0.5,
        "max_angular": 0.8,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/local_grid_obstacle",
                description="D435i RTAB-Map obstacle cloud topic.",
            ),
            Node(
                package="go2_uwb",
                executable="obstacle_detector_node",
                name="obstacle_detector_node",
                output="screen",
                parameters=[
                    {
                        "cloud_topic": pointcloud_topic,
                        "target_frame": "base_footprint",
                        "distance_topic": "/obstacle/nearest_distance",
                        "avoid_vector_topic": "/obstacle/avoid_vector",
                        "debug_cloud_topic": "/obstacle/used_points",
                        "publish_debug_cloud": True,
                        "tf_timeout_sec": 0.05,
                        "enable_passthrough_filter": True,
                        "enable_voxel_filter": True,
                        "voxel_leaf_size": 0.05,
                        "enable_radius_outlier_filter": False,
                        "radius_search": 0.12,
                        "min_neighbors_in_radius": 3,
                        "min_x": 0.2,
                        "max_x": 2.0,
                        "max_abs_y": 0.8,
                        "min_z": 0.08,
                        "max_z": 1.0,
                        "max_avoid_angular": 0.8,
                        "min_obstacle_points": 8,
                        "side_count_deadband": 3,
                    }
                ],
            ),
            Node(
                package="go2_uwb",
                executable="follow_controller_node",
                name="follow_controller_node",
                output="screen",
                parameters=[
                    {
                        "target_topic": "/uwb/target_point",
                        "obstacle_distance_topic": "/obstacle/nearest_distance",
                        "avoid_vector_topic": "/obstacle/avoid_vector",
                        "cmd_vel_topic": cmd_vel_topic,
                        "status_topic": "/go2_uwb/controller_status",
                        "control_rate": 20.0,
                        "status_rate": 2.0,
                        "target_distance": 1.5,
                        "target_deadband": 0.12,
                        "angle_deadband": 0.08,
                        "max_target_jump": 0.7,
                        "avoid_distance": 0.9,
                        "avoid_release_distance": 1.05,
                        "front_stop_distance": 0.45,
                        "linear_k": 0.4,
                        "angular_k": 1.0,
                        "uwb_timeout": 1.0,
                        "obstacle_timeout": 0.7,
                        "target_filter_alpha": 0.35,
                        "avoid_angular_filter_alpha": 0.4,
                        "max_linear_accel": 0.4,
                        "max_angular_accel": 1.2,
                        "turn_slowdown_angle": 0.8,
                        "min_turn_slowdown": 0.35,
                        **common_speed_limits,
                    }
                ],
            ),
        ]
    )
