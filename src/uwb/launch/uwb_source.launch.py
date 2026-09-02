from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# 启动 UWB 串口驱动，只发布厂家原始消息；目标坐标适配由上层跟随包负责。
def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    frame_id = LaunchConfiguration("frame_id")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    aoa_frequency_hz = LaunchConfiguration("aoa_frequency_hz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("frame_id", default_value="uwb_link"),
            DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("aoa_frequency_hz", default_value="10"),
            Node(
                package="uwb_aoa_pkg",
                executable="libAoa_robot_example",
                name="libAoa_robot_publisher",
                output="screen",
                arguments=[serial_port],
                parameters=[
                    {
                        "frame_id": frame_id,
                        "publish_rate_hz": ParameterValue(
                            publish_rate_hz,
                            value_type=float,
                        ),
                        "aoa_frequency_hz": ParameterValue(
                            aoa_frequency_hz,
                            value_type=int,
                        ),
                    }
                ],
            ),
        ]
    )
