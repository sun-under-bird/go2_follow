"""仿真与真实数据接入共用的 ROS 接口，算法调参仍来自原配置。"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RosInterface:
    use_sim_time: bool = True
    odom_topic: str = '/follow_demo/odom'
    target_topic: str = '/uwb/target_point'
    depth_topic: str = '/camera/depth/image_rect_raw'
    camera_info_topic: str = '/camera/depth/camera_info'
    operator_topic: str = '/follow_demo/operator'
    command_topic: str = '/cmd_vel'
    status_topic: str = '/follow_demo/control_status'
    odom_frame: str = 'odom'
    base_frame: str = 'base_footprint'
    depth_frame: str = 'camera_left_optical'
    obstacle_cloud_topic: str = ''
    automatic_follow: bool = False


def yaw_from_quaternion(quaternion):
    """从 wxyz 四元数提取世界系偏航角，无步态模型依赖。"""
    w, x, y, z = quaternion
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def hardware_interface(odom_frame, base_frame, depth_frame, command_topic='/cmd_vel'):
    """真实数据先经过检查/转换，最终速度送往已有底盘桥的订阅话题。"""
    return RosInterface(use_sim_time=False, odom_topic='/go2_follow/input/odom',
                        target_topic='/go2_follow/input/target', depth_topic='/go2_follow/input/depth',
                        camera_info_topic='/go2_follow/input/camera_info', operator_topic='',
                        command_topic=command_topic, status_topic='/go2_follow/control_status',
                        odom_frame=odom_frame, base_frame=base_frame, depth_frame=depth_frame,
                        obstacle_cloud_topic='/go2_follow/input/obstacles', automatic_follow=True)
