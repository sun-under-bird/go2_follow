from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """启动拼接双目相机、拆图、矫正、视差和点云链路。"""
    video_device = LaunchConfiguration("video_device")
    left_info_url = LaunchConfiguration("left_info_url")
    right_info_url = LaunchConfiguration("right_info_url")
    max_publish_rate_hz = LaunchConfiguration("max_publish_rate_hz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("video_device", default_value="/dev/video0"),
            DeclareLaunchArgument("left_info_url", default_value=""),
            DeclareLaunchArgument("right_info_url", default_value=""),
            DeclareLaunchArgument("max_publish_rate_hz", default_value="40.0"),
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="stitched_stereo_v4l2",
                output="screen",
                parameters=[
                    {
                        "video_device": video_device,
                        "pixel_format": "YUYV",
                        "image_size": [1280, 480],
                        "output_encoding": "yuv422_yuy2",
                        "camera_frame_id": "stitched_stereo_camera",
                    }
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="go2_stereo_camera",
                executable="stereo_split_node",
                name="stereo_split_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/image_raw",
                        "left_image_topic": "/stereo/left/camera/image_mono",
                        "right_image_topic": "/stereo/right/camera/image_mono",
                        "left_info_topic": "/stereo/left/camera/camera_info",
                        "right_info_topic": "/stereo/right/camera/camera_info",
                        "left_info_url": left_info_url,
                        "right_info_url": right_info_url,
                        "left_frame": "left_camera_optical_frame",
                        "right_frame": "right_camera_optical_frame",
                        "output_encoding": "mono8",
                        "max_publish_rate_hz": ParameterValue(
                            max_publish_rate_hz,
                            value_type=float,
                        ),
                    }
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="image_proc",
                executable="rectify_node",
                name="rectify_left",
                namespace="/stereo/left/camera",
                output="screen",
                parameters=[
                    {
                        "qos_overrides./stereo/left/camera/image_mono.subscription.reliability": "best_effort",
                        "qos_overrides./stereo/left/camera/camera_info.subscription.reliability": "best_effort",
                        "qos_overrides./stereo/left/camera/image_rect.publisher.reliability": "best_effort",
                    }
                ],
                remappings=[
                    ("image", "/stereo/left/camera/image_mono"),
                    ("camera_info", "/stereo/left/camera/camera_info"),
                    ("image_rect", "/stereo/left/camera/image_rect"),
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="image_proc",
                executable="rectify_node",
                name="rectify_right",
                namespace="/stereo/right/camera",
                output="screen",
                parameters=[
                    {
                        "qos_overrides./stereo/right/camera/image_mono.subscription.reliability": "best_effort",
                        "qos_overrides./stereo/right/camera/camera_info.subscription.reliability": "best_effort",
                        "qos_overrides./stereo/right/camera/image_rect.publisher.reliability": "best_effort",
                    }
                ],
                remappings=[
                    ("image", "/stereo/right/camera/image_mono"),
                    ("camera_info", "/stereo/right/camera/camera_info"),
                    ("image_rect", "/stereo/right/camera/image_rect"),
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="stereo_image_proc",
                executable="disparity_node",
                name="stereo_disparity",
                output="screen",
                parameters=[
                    {
                        "qos_overrides./stereo/left/camera/image_rect.subscription.reliability": "best_effort",
                        "qos_overrides./stereo/right/camera/image_rect.subscription.reliability": "best_effort",
                        "qos_overrides./stereo/left/camera/camera_info.subscription.reliability": "best_effort",
                        "qos_overrides./stereo/right/camera/camera_info.subscription.reliability": "best_effort",
                        "qos_overrides./stereo/disparity.publisher.reliability": "best_effort",
                    }
                ],
                remappings=[
                    ("left/image_rect", "/stereo/left/camera/image_rect"),
                    ("left/camera_info", "/stereo/left/camera/camera_info"),
                    ("right/image_rect", "/stereo/right/camera/image_rect"),
                    ("right/camera_info", "/stereo/right/camera/camera_info"),
                    ("disparity", "/stereo/disparity"),
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="stereo_image_proc",
                executable="point_cloud_node",
                name="stereo_point_cloud",
                output="screen",
                parameters=[
                    {
                        "qos_overrides./stereo/disparity.subscription.reliability": "best_effort",
                        "qos_overrides./stereo/left/camera/image_rect.subscription.reliability": "best_effort",
                        "qos_overrides./stereo/left/camera/camera_info.subscription.reliability": "best_effort",
                        "qos_overrides./stereo/right/camera/camera_info.subscription.reliability": "best_effort",
                        "qos_overrides./stereo/points2.publisher.reliability": "best_effort",
                    }
                ],
                remappings=[
                    ("disparity", "/stereo/disparity"),
                    ("left/image_rect_color", "/stereo/left/camera/image_rect"),
                    ("left/camera_info", "/stereo/left/camera/camera_info"),
                    ("right/camera_info", "/stereo/right/camera/camera_info"),
                    ("points2", "/stereo/points2"),
                ],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
        ]
    )
