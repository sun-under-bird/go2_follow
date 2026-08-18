from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    frame_id = LaunchConfiguration("frame_id")
    target_frame = LaunchConfiguration("target_frame")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    use_raw_fields = LaunchConfiguration("use_raw_fields")
    invert_y = LaunchConfiguration("invert_y")
    target_point_topic = LaunchConfiguration("target_point_topic")
    start_driver = LaunchConfiguration("start_driver")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("frame_id", default_value="uwb_link"),
            DeclareLaunchArgument("target_frame", default_value="base_footprint"),
            DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("use_raw_fields", default_value="false"),
            DeclareLaunchArgument("invert_y", default_value="false"),
            DeclareLaunchArgument("target_point_topic", default_value="/uwb/target_point"),
            DeclareLaunchArgument("start_driver", default_value="true"),
            Node(
                package="uwb_aoa_pkg",
                executable="libAoa_robot_example",
                name="libAoa_robot_publisher",
                output="screen",
                condition=IfCondition(start_driver),
                arguments=[serial_port, "--ros-args", "--log-level", "warn"],
                parameters=[
                    {
                        "frame_id": frame_id,
                        "publish_rate_hz": publish_rate_hz,
                    }
                ],
            ),
            Node(
                package="uwb_aoa_pkg",
                executable="one1000_target_point_node",
                name="one1000_target_point_node",
                output="screen",
                arguments=["--ros-args", "--log-level", "warn"],
                parameters=[
                    {
                        "one1000_topic": "/libAoa_robot_publisher",
                        "target_point_topic": target_point_topic,
                        "target_frame": target_frame,
                        "one1000_frame": frame_id,
                        "use_tf": True,
                        "use_latest_tf": True,
                        "transform_timeout_sec": 0.2,
                        "prefer_xy": True,
                        "use_raw_fields": use_raw_fields,
                        "angle_in_degrees": False,
                        "invert_y": invert_y,
                        "angle_offset_rad": 0.0,
                        "anchor_x_offset": 0.0,
                        "anchor_y_offset": 0.0,
                        "require_valid_state": True,
                        "min_target_distance_m": 0.2,
                        "max_target_distance_m": 8.0,
                    }
                ],
            ),
        ]
    )
