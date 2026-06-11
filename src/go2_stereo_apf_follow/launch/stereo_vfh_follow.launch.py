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
    """生成 UWB + 双目点云 VFH 跟随避障启动描述。"""
    pkg_share = get_package_share_directory("go2_stereo_apf_follow")
    default_config = os.path.join(pkg_share, "config", "stereo_vfh_follow.yaml")

    config_file = LaunchConfiguration("config_file")
    base_frame = LaunchConfiguration("base_frame")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    cmd_vel_out = LaunchConfiguration("cmd_vel_out")
    start_go2_driver = LaunchConfiguration("start_go2_driver")
    start_twist_bridge = LaunchConfiguration("start_twist_bridge")
    start_camera = LaunchConfiguration("start_camera")
    start_rtabmap = LaunchConfiguration("start_rtabmap")
    start_uwb = LaunchConfiguration("start_uwb")
    start_seed = LaunchConfiguration("start_seed")
    start_controller = LaunchConfiguration("start_controller")
    uwb_device = LaunchConfiguration("uwb_device")
    uwb_frame_id = LaunchConfiguration("uwb_frame_id")
    uwb_publish_rate_hz = LaunchConfiguration("uwb_publish_rate_hz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    camera_launch = PathJoinSubstitution(
        [FindPackageShare("stereo_camera_pkg"), "launch", "usb_400.launch.py"]
    )
    rtabmap_launch = PathJoinSubstitution(
        [FindPackageShare("go2_dynamic_follow_avoidance"), "launch", "d435i_rtabmap.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("pointcloud_topic", default_value="/local_grid_obstacle"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("cmd_vel_out", default_value="/cmd_vel"),
            DeclareLaunchArgument("start_go2_driver", default_value="false"),
            DeclareLaunchArgument("start_twist_bridge", default_value="false"),
            DeclareLaunchArgument("start_camera", default_value="false"),
            DeclareLaunchArgument("start_rtabmap", default_value="false"),
            DeclareLaunchArgument("start_uwb", default_value="false"),
            DeclareLaunchArgument("start_seed", default_value="true"),
            DeclareLaunchArgument("start_controller", default_value="true"),
            DeclareLaunchArgument("uwb_device", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("uwb_frame_id", default_value="uwb_link"),
            DeclareLaunchArgument("uwb_publish_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
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
                remappings=[("cmd_vel", cmd_vel_out)],
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
                package="go2_stereo_apf_follow",
                executable="uwb_target_seed_node",
                name="uwb_target_seed_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "base_frame": base_frame,
                        "one1000_frame": uwb_frame_id,
                        "use_sim_time": use_sim_time,
                    },
                ],
                condition=IfCondition(start_seed),
            ),
            Node(
                package="go2_stereo_apf_follow",
                executable="stereo_vfh_controller_node",
                name="stereo_vfh_controller_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "base_frame": base_frame,
                        "pointcloud_topic": pointcloud_topic,
                        "cmd_vel_topic": cmd_vel_topic,
                        "use_sim_time": use_sim_time,
                    },
                ],
                condition=IfCondition(start_controller),
            ),
        ]
    )
