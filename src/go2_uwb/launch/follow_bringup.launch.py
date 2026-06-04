from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动 Go2 UWB 跟随和双目局部避障的 C++ 节点链路。"""
    common_speed_limits = {
        'max_linear': 0.5,
        'max_angular': 0.8,
    }

    return LaunchDescription([
        # UWB 坐标过滤节点：只转发 state == 1 的主人坐标。
        Node(
            package='go2_uwb',
            executable='uwb_filter_node',
            name='uwb_filter_node',
            output='screen',
            parameters=[{
                'input_topic': '/libAoa_robot_publisher',
                'output_topic': '/uwb/target_point',
                'output_frame': 'uwb_link',
            }],
        ),

        # 左目图像校正节点：把左目原始灰度图校正为立体匹配需要的 image_rect。
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_left',
            namespace='/stereo/left/camera',
            parameters=[{
                'qos_overrides./stereo/left/camera/image_mono.subscription.reliability':
                    'best_effort',
                'qos_overrides./stereo/left/camera/camera_info.subscription.reliability':
                    'best_effort',
                'qos_overrides./stereo/left/camera/image_rect.publisher.reliability':
                    'best_effort',
            }],
            remappings=[
                ('image', '/stereo/left/camera/image_mono'),
                ('camera_info', '/stereo/left/camera/camera_info'),
                ('image_rect', '/stereo/left/camera/image_rect'),
            ],
        ),

        # 右目图像校正节点：把右目原始灰度图校正为立体匹配需要的 image_rect。
        Node(
            package='image_proc',
            executable='rectify_node',
            name='rectify_right',
            namespace='/stereo/right/camera',
            parameters=[{
                'qos_overrides./stereo/right/camera/image_mono.subscription.reliability':
                    'best_effort',
                'qos_overrides./stereo/right/camera/camera_info.subscription.reliability':
                    'best_effort',
                'qos_overrides./stereo/right/camera/image_rect.publisher.reliability':
                    'best_effort',
            }],
            remappings=[
                ('image', '/stereo/right/camera/image_mono'),
                ('camera_info', '/stereo/right/camera/camera_info'),
                ('image_rect', '/stereo/right/camera/image_rect'),
            ],
        ),

        # 视差计算节点：根据左右校正图和相机内参发布 /stereo/disparity。
        Node(
            package='stereo_image_proc',
            executable='disparity_node',
            name='disparity_node',
            output='screen',
            remappings=[
                ('left/image_rect', '/stereo/left/camera/image_rect'),
                ('left/camera_info', '/stereo/left/camera/camera_info'),
                ('right/image_rect', '/stereo/right/camera/image_rect'),
                ('right/camera_info', '/stereo/right/camera/camera_info'),
                ('disparity', '/stereo/disparity'),
            ],
        ),

        # 点云生成节点：根据视差图生成 /stereo/points2 原始双目点云。
        Node(
            package='stereo_image_proc',
            executable='point_cloud_node',
            name='point_cloud',
            output='screen',
            remappings=[
                ('disparity', '/stereo/disparity'),
                ('left/image_rect_color', '/stereo/left/camera/image_rect'),
                ('left/camera_info', '/stereo/left/camera/camera_info'),
                ('right/camera_info', '/stereo/right/camera/camera_info'),
                ('points2', '/stereo/points2'),
            ],
        ),

        # PCL 点云避障节点：将 /stereo/points2 转到 base_link 后做高度过滤和孤立点过滤。
        Node(
            package='go2_uwb',
            executable='obstacle_detector_node',
            name='obstacle_detector_node',
            output='screen',
            parameters=[{
                'cloud_topic': '/stereo/points2',
                'target_frame': 'base_link',
                'distance_topic': '/obstacle/nearest_distance',
                'avoid_vector_topic': '/obstacle/avoid_vector',
                'debug_cloud_topic': '/obstacle/used_points',
                'publish_debug_cloud': True,
                'tf_timeout_sec': 0.05,
                'enable_passthrough_filter': True,
                'enable_radius_outlier_filter': True,
                'radius_search': 0.12,
                'min_neighbors_in_radius': 3,
                'min_x': 0.2,
                'max_x': 2.0,
                'max_abs_y': 0.8,
                'min_z': 0.08,
                'max_z': 1.0,
                'max_avoid_angular': 0.8,
                'min_obstacle_points': 8,
                'side_count_deadband': 3,
            }],
        ),

        # 跟随控制节点：融合 UWB 目标和避障建议，直接发布 /cmd_vel 给已有 go2_twist_bridge。
        Node(
            package='go2_uwb',
            executable='follow_controller_node',
            name='follow_controller_node',
            output='screen',
            parameters=[{
                'target_topic': '/uwb/target_point',
                'obstacle_distance_topic': '/obstacle/nearest_distance',
                'avoid_vector_topic': '/obstacle/avoid_vector',
                'cmd_vel_topic': '/cmd_vel',
                'status_topic': '/go2_uwb/controller_status',
                'control_rate': 20.0,
                'status_rate': 2.0,
                'target_distance': 1.5,
                'target_deadband': 0.12,
                'angle_deadband': 0.08,
                'max_target_jump': 0.7,
                'avoid_distance': 0.9,
                'avoid_release_distance': 1.05,
                'front_stop_distance': 0.45,
                'linear_k': 0.4,
                'angular_k': 1.0,
                'uwb_timeout': 1.0,
                'obstacle_timeout': 0.7,
                'target_filter_alpha': 0.35,
                'avoid_angular_filter_alpha': 0.4,
                'max_linear_accel': 0.4,
                'max_angular_accel': 1.2,
                'turn_slowdown_angle': 0.8,
                'min_turn_slowdown': 0.35,
                **common_speed_limits,
            }],
        ),
    ])
