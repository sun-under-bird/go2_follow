from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    uwb_params_file = LaunchConfiguration("uwb_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    cmd_vel_nav = LaunchConfiguration("cmd_vel_nav")
    cmd_vel_out = LaunchConfiguration("cmd_vel_out")
    start_stereo_camera = LaunchConfiguration("start_stereo_camera")
    start_stereo_cloud_filter = LaunchConfiguration("start_stereo_cloud_filter")
    video_device = LaunchConfiguration("video_device")
    left_info_url = LaunchConfiguration("left_info_url")
    right_info_url = LaunchConfiguration("right_info_url")

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
    default_stereo_camera_launch = PathJoinSubstitution(
        [
            FindPackageShare("go2_stereo_camera"),
            "launch",
            "stereo_camera.launch.py",
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
                "uwb_params_file",
                default_value=default_uwb_params_file,
                description="Parameter file for the UWB target and direct MPPI follow nodes.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params_file,
                description="Parameter file for Nav2 controller, local costmap, and velocity smoother.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_nav",
                default_value="/cmd_vel_nav",
                description="Raw velocity topic from Nav2 controller_server.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_out",
                default_value="/cmd_vel_safe",
                description="Smoothed velocity topic used by the Go2 velocity bridge.",
            ),
            DeclareLaunchArgument(
                "start_stereo_camera",
                default_value="false",
                description="Start v4l2 stitched stereo camera splitting and point cloud pipeline.",
            ),
            DeclareLaunchArgument(
                "start_stereo_cloud_filter",
                default_value="true",
                description="Filter /stereo/points2 into local obstacle and ground clouds.",
            ),
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/video0",
                description="Linux v4l2 device for the stitched stereo camera.",
            ),
            DeclareLaunchArgument(
                "left_info_url",
                default_value="",
                description="Left camera calibration YAML path.",
            ),
            DeclareLaunchArgument(
                "right_info_url",
                default_value="",
                description="Right camera calibration YAML path.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(default_stereo_camera_launch),
                launch_arguments={
                    "video_device": video_device,
                    "left_info_url": left_info_url,
                    "right_info_url": right_info_url,
                }.items(),
                condition=IfCondition(start_stereo_camera),
            ),
            Node(
                package="go2_uwb_mppi_follow",
                executable="stereo_cloud_filter_node",
                name="stereo_cloud_filter_node",
                output="screen",
                parameters=[
                    uwb_params_file,
                    {"use_sim_time": use_sim_time},
                ],
                condition=IfCondition(start_stereo_cloud_filter),
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
                remappings=[("cmd_vel", cmd_vel_nav)],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("cmd_vel", cmd_vel_nav),
                    ("cmd_vel_smoothed", cmd_vel_out),
                ],
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
                            "velocity_smoother",
                        ]
                    },
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="go2_uwb_mppi_follow",
                executable="one1000_target_point_node",
                name="one1000_target_point_node",
                output="screen",
                parameters=[
                    uwb_params_file,
                    {"use_sim_time": use_sim_time},
                ],
                arguments=["--ros-args", "--log-level", "warn"],
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
                    {"stop_cmd_vel_topic": cmd_vel_nav},
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
        ]
    )
