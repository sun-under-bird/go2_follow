from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _stereo_remaps(left_image, right_image, left_info, right_info, odom_topic):
    return [
        ("left/image_rect", left_image),
        ("right/image_rect", right_image),
        ("left/camera_info", left_info),
        ("right/camera_info", right_info),
        ("odom", odom_topic),
    ]


def generate_launch_description():
    base_frame = LaunchConfiguration("base_frame")
    odom_topic = LaunchConfiguration("odom_topic")
    localization = LaunchConfiguration("localization")
    use_viz = LaunchConfiguration("use_viz")
    use_stereo_odometry = LaunchConfiguration("use_stereo_odometry")
    database_path = LaunchConfiguration("database_path")
    slam_mode = PythonExpression(["'", localization, "' == 'false'"])

    left_image = LaunchConfiguration("left_image")
    right_image = LaunchConfiguration("right_image")
    left_camera_info = LaunchConfiguration("left_camera_info")
    right_camera_info = LaunchConfiguration("right_camera_info")

    publish_base_to_camera_tf = LaunchConfiguration("publish_base_to_camera_tf")
    camera_frame = LaunchConfiguration("camera_frame")
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")

    rtabmap_odom_params = {
        "frame_id": base_frame,
        "subscribe_rgbd": False,
        "subscribe_stereo": True,
        "subscribe_odom_info": True,
        "use_sim_time": False,
        "approx_sync": True,
        "approx_sync_max_interval": 0.1,
        "sync_queue_size": 10,
        "topic_queue_size": 5,
        "wait_for_transform": 0.5,
        "Rtabmap/ImagesAlreadyRectified": "true",
        "publish_tf": False,
        "Vis/FeatureType": "8",
        "Vis/EstimationType": "1",
        "Vis/MinInliers": "12",
        "Vis/MaxFeatures": "1000",
        "Vis/CorType": "0",
        "Odom/ResetCountdown": "5",
        "Odom/Strategy": "0",
        "OdomF2M/MaxSize": "1000",
        "GFTT/MinDistance": "5",
        "GFTT/QualityLevel": "0.00001",
        "Stereo/MaxDisparity": "256",
        "wait_imu_to_init": False,
        "qos": 2,
    }

    rtabmap_grid_filter_params = {
        "Grid/3D": "true",
        "Grid/RayTracing": "true",
        "Grid/RangeMin": "0.05",
        "Grid/RangeMax": "3.0",
        "Grid/CellSize": "0.05",
        "Grid/NormalsSegmentation": "true",
        "Grid/MaxGroundAngle": "25",
        "Grid/NormalK": "12",
        "Grid/ClusterRadius": "0.15",
        "Grid/MinClusterSize": "12",
        "Grid/FlatObstacleDetected": "true",
        "Grid/GroundIsObstacle": "false",
        "Grid/MinObstacleHeight": "0.02",
        "Grid/MaxObstacleHeight": "0.8",
        "Grid/NoiseFilteringRadius": "0.12",
        "Grid/NoiseFilteringMinNeighbors": "4",
        "Grid/MapFrameProjection": "false",
    }

    rtabmap_slam_params = {
        "frame_id": base_frame,
        "subscribe_rgbd": False,
        "subscribe_stereo": True,
        "subscribe_odom_info": False,
        "subscribe_odom": True,
        "odom_frame_id": "odom",
        "use_sim_time": False,
        "approx_sync": True,
        "approx_sync_max_interval": 0.1,
        "sync_queue_size": 10,
        "topic_queue_size": 5,
        "wait_for_transform": 0.5,
        "tf_delay": 0.05,
        "Rtabmap/ImagesAlreadyRectified": "true",
        "Rtabmap/DetectionRate": "0",
        "Reg/Force3DoF": "true",
        "Kp/MaxFeatures": "1000",
        "Kp/NndrRatio": "0.75",
        "GFTT/MinDistance": "5",
        "GFTT/QualityLevel": "0.00001",
        "GFTT/MaxCorners": "800",
        "Stereo/MaxDisparity": "256",
        "qos": 2,
        **rtabmap_grid_filter_params,
    }

    rtabmap_localization_params = {
        "Mem/IncrementalMemory": "False",
        "Mem/InitWMWithAllNodes": "True",
        "RGBD/LocalizationSmoothing": "true",
        "RGBD/LocalizationPriorError": "0.001",
        "RGBD/MaxOdomCacheSize": "10",
    }

    stereo_remaps = _stereo_remaps(
        left_image,
        right_image,
        left_camera_info,
        right_camera_info,
        odom_topic,
    )
    odom_remaps = _stereo_remaps(
        left_image,
        right_image,
        left_camera_info,
        right_camera_info,
        "/vo",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("odom_topic", default_value="/odom_leg"),
            DeclareLaunchArgument("localization", default_value="false"),
            DeclareLaunchArgument("use_viz", default_value="false"),
            DeclareLaunchArgument("use_stereo_odometry", default_value="false"),
            DeclareLaunchArgument("database_path", default_value="~/.ros/d435i_rtabmap.db"),
            DeclareLaunchArgument(
                "left_image",
                default_value="/camera/camera/infra1/image_rect_raw",
            ),
            DeclareLaunchArgument(
                "right_image",
                default_value="/camera/camera/infra2/image_rect_raw",
            ),
            DeclareLaunchArgument(
                "left_camera_info",
                default_value="/camera/camera/infra1/camera_info",
            ),
            DeclareLaunchArgument(
                "right_camera_info",
                default_value="/camera/camera/infra2/camera_info",
            ),
            DeclareLaunchArgument("publish_base_to_camera_tf", default_value="false"),
            DeclareLaunchArgument("camera_frame", default_value="camera_link"),
            DeclareLaunchArgument("camera_x", default_value="0.0"),
            DeclareLaunchArgument("camera_y", default_value="0.0"),
            DeclareLaunchArgument("camera_z", default_value="0.0"),
            DeclareLaunchArgument("camera_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_yaw", default_value="0.0"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_d435i_tf",
                output="screen",
                arguments=[
                    camera_x,
                    camera_y,
                    camera_z,
                    camera_yaw,
                    camera_pitch,
                    camera_roll,
                    base_frame,
                    camera_frame,
                ],
                condition=IfCondition(publish_base_to_camera_tf),
            ),
            Node(
                package="rtabmap_odom",
                executable="stereo_odometry",
                name="d435i_stereo_odometry",
                output="screen",
                parameters=[rtabmap_odom_params],
                remappings=odom_remaps,
                arguments=["--ros-args", "--log-level", "warn"],
                condition=IfCondition(use_stereo_odometry),
            ),
            Node(
                condition=IfCondition(slam_mode),
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[rtabmap_slam_params, {"database_path": database_path}],
                remappings=stereo_remaps,
                arguments=["--ros-args", "--log-level", "warn", "--", "-d"],
            ),
            Node(
                condition=IfCondition(localization),
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[
                    rtabmap_slam_params,
                    rtabmap_localization_params,
                    {"database_path": database_path},
                ],
                remappings=stereo_remaps,
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                name="rtabmap_viz",
                output="screen",
                condition=IfCondition(use_viz),
                parameters=[rtabmap_slam_params],
                remappings=stereo_remaps,
                arguments=["--ros-args", "--log-level", "warn"],
            ),
        ]
    )
