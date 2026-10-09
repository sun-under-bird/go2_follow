"""真实传感器运行入口：复用原算法，将最终 Twist 速度送往已有底盘桥。"""
import argparse
import copy
import math
import threading
import time
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time as RosTime
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import TransformException

from .executor_pulse import ExecutorPulse
from .hardware_contract import (cloud_xyz, depth_in_meters, mark_obstacles, transform_point,
                                transform_points, valid_stamp, validate_odometry)
from .mppi_runtime import MppiRuntime
from .ros_contract import hardware_interface
from .ros_nodes import FollowerNode, TfReceiver


class HardwareInputs(Node):
    """接入标准真实消息；可靠转发里程计满足 Humble Nav2 的订阅要求。"""
    def __init__(self, args, interface, tf_buffer):
        super().__init__('go2_follow_hardware_inputs')
        self.args, self.interface, self.tf_buffer = args, interface, tf_buffer
        self.info, self.last_warning = None, -math.inf
        self.depths, self.clouds, self.pending_clouds, self.last_pair = {}, {}, {}, -1
        self.odom_pub = self.create_publisher(Odometry, interface.odom_topic, 5)
        self.target_pub = self.create_publisher(PointStamped, interface.target_topic, qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, interface.depth_topic, qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, interface.camera_info_topic, qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(PointCloud2, interface.obstacle_cloud_topic, qos_profile_sensor_data)
        for source, internal in ((args.odom_topic, interface.odom_topic), (args.target_topic, interface.target_topic),
                                 (args.depth_topic, interface.depth_topic), (args.camera_info_topic, interface.camera_info_topic),
                                 (args.obstacle_topic, interface.obstacle_cloud_topic)):
            if source == internal:
                raise ValueError('hardware source must differ from internal relay topic')
        self.create_subscription(Odometry, args.odom_topic, self.on_odometry, qos_profile_sensor_data)
        self.create_subscription(PointStamped, args.target_topic, self.on_target, qos_profile_sensor_data)
        self.create_subscription(Image, args.depth_topic, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.camera_info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, args.obstacle_topic, self.on_cloud, qos_profile_sensor_data)
        self.create_timer(.02, self.retry_clouds)

    @staticmethod
    def stamp_key(message):
        return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec

    def publish_pair(self, key):
        if key > self.last_pair and key in self.depths and key in self.clouds:
            output, info = self.depths[key]
            self.cloud_pub.publish(self.clouds[key])
            self.info_pub.publish(info)
            self.depth_pub.publish(output)
            self.last_pair = key
        for cache in (self.depths, self.clouds, self.pending_clouds):
            for old in list(cache):
                if old <= self.last_pair:
                    del cache[old]
            while len(cache) > 16:
                del cache[min(cache)]

    def warn(self, error):
        now = time.monotonic()
        if now - self.last_warning >= 2:
            self.get_logger().warning(str(error))
            self.last_warning = now

    def on_odometry(self, message):
        try:
            validate_odometry(message, self.interface.odom_frame, self.interface.base_frame)
        except ValueError as error:
            self.warn(error)
            return
        self.odom_pub.publish(message)

    def on_target(self, message):
        try:
            if not valid_stamp(message.header.stamp) or not message.header.frame_id:
                raise ValueError('UWB target requires an acquisition stamp and frame')
            point = message.point
            if not all(math.isfinite(value) for value in (point.x, point.y, point.z)):
                raise ValueError('UWB target contains nonfinite values')
            if message.header.frame_id != self.interface.base_frame:
                tf = self.tf_buffer.lookup_transform(self.interface.base_frame, message.header.frame_id,
                                                     RosTime.from_msg(message.header.stamp)).transform
                xyz = transform_point(point, tf)
                message = copy.deepcopy(message)
                message.header.frame_id = self.interface.base_frame
                message.point.x, message.point.y, message.point.z = map(float, xyz)
        except (ValueError, TransformException) as error:
            self.warn(error)
            return
        self.target_pub.publish(message)

    def on_info(self, message):
        if (message.header.frame_id != self.interface.depth_frame or message.width <= 0 or message.height <= 0
                or not all(math.isfinite(value) for value in message.k) or message.k[0] <= 0 or message.k[4] <= 0):
            self.warn('CameraInfo must match the configured rectified depth optical frame and valid intrinsics')
            return
        self.info = message
        self.info_pub.publish(message)

    def on_depth(self, message):
        try:
            if message.header.frame_id != self.interface.depth_frame or not valid_stamp(message.header.stamp):
                raise ValueError('depth requires its real optical frame and acquisition timestamp')
            if self.info is None or (self.info.width, self.info.height) != (message.width, message.height):
                raise ValueError('matching CameraInfo is missing')
            depth = depth_in_meters(message.data, message.width, message.height, message.step, message.encoding,
                                    bool(message.is_bigendian), self.args.integer_depth_scale)
            output = Image()
            output.header = message.header
            output.width, output.height = message.width, message.height
            output.encoding, output.is_bigendian, output.step = '32FC1', 0, message.width * 4
            output.data = depth.tobytes()
        except ValueError as error:
            self.warn(error)
            return
        key = self.stamp_key(message)
        self.depths[key] = (output, self.info)
        self.publish_pair(key)

    def on_cloud(self, message):
        try:
            if not valid_stamp(message.header.stamp) or not message.header.frame_id:
                raise ValueError('obstacle cloud requires an acquisition stamp and frame')
            points = cloud_xyz(message)
        except ValueError as error:
            self.warn(error)
            return
        key = self.stamp_key(message)
        if key > self.last_pair:
            self.pending_clouds[key] = (message.header, points)
        self.publish_pair(key)
        self.retry_clouds()

    def retry_clouds(self):
        # TF 接收在独立执行器；允许同帧变换晚于点云到达，不改用最新 TF 或重写采集时间。
        for key in sorted(self.pending_clouds):
            header, points = self.pending_clouds[key]
            try:
                tf = self.tf_buffer.lookup_transform(self.interface.odom_frame, header.frame_id,
                                                     RosTime.from_msg(header.stamp)).transform
                points = transform_points(points, tf)
            except TransformException:
                continue
            except ValueError as error:
                del self.pending_clouds[key]
                self.warn(error)
                continue
            output = PointCloud2()
            output.header = copy.deepcopy(header)
            output.header.frame_id = self.interface.odom_frame
            output.height, output.width = 1, len(points)
            output.fields = [PointField(name=name, offset=index*4, datatype=PointField.FLOAT32, count=1)
                             for index, name in enumerate(('x', 'y', 'z'))]
            output.is_bigendian, output.is_dense = False, True
            output.point_step, output.row_step = 12, len(points)*12
            output.data = np.asarray(points, dtype='<f4').tobytes()
            del self.pending_clouds[key]
            self.clouds[key] = output
            self.publish_pair(key)


class HardwareFollower(FollowerNode):
    """接入同帧障碍点，按实际传感器健康自动运行。"""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.obstacles = {}
        self.create_subscription(PointCloud2, self.interface.obstacle_cloud_topic, self.on_obstacles, qos_profile_sensor_data)

    def on_obstacles(self, message):
        if message.header.frame_id != self.interface.odom_frame:
            return
        key = HardwareInputs.stamp_key(message)
        try:
            self.obstacles[key] = cloud_xyz(message)
        except ValueError as error:
            self.get_logger().error(f'obstacle frame rejected: {error}')
            return
        while len(self.obstacles) > 16:
            del self.obstacles[min(self.obstacles)]

    def integrate_depth(self, now):
        self.accept_depth_result(now)
        if self.depth_future is not None:
            self.depth_submit_reason = 'WORKER_BUSY'
            return
        message = self.pending_depth
        if message is not None:
            key = HardwareInputs.stamp_key(message)
            if key not in self.obstacles:
                self.depth_submit_reason = 'WAIT_MATCHING_OBSTACLE_FRAME'
                return
            grid = self.core.grid
            grid.hardware_obstacles = self.obstacles[key]
            try:
                super().integrate_depth(now)
            finally:
                del grid.hardware_obstacles
        else:
            super().integrate_depth(now)

    @staticmethod
    def fuse_depth(snapshot, depth, intrinsic, rotation, translation, stamp):
        started = time.monotonic()
        snapshot, integrated, _ = FollowerNode.fuse_depth(snapshot, depth, intrinsic, rotation, translation, stamp)
        if integrated:
            mark_obstacles(snapshot, snapshot.hardware_obstacles, stamp)
        del snapshot.hardware_obstacles
        return snapshot, integrated, (time.monotonic() - started) * 1000

def argument_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--target-topic', default='/uwb/target_point')
    parser.add_argument('--depth-topic', default='/stereo/depth_debug')
    parser.add_argument('--camera-info-topic', default='/camera/camera/infra1/camera_info')
    parser.add_argument('--obstacle-topic', default='/local_grid_obstacle')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel',
                        help='final geometry_msgs/Twist output consumed by the existing chassis bridge')
    parser.add_argument('--depth-frame', required=True, help='actual optical frame from depth/CameraInfo headers')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--integer-depth-scale', type=float, default=.001, help='meters per 16UC1 unit; verify driver')
    parser.add_argument('--duration', type=float, default=0., help='wall seconds; 0 runs until interrupted')
    return parser


def spin_until_shutdown(executor):
    """ROS 的 SIGINT 会关闭共享 context；执行器线程正常退出。"""
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main(argv=None):
    args = argument_parser().parse_args(argv)
    if not math.isfinite(args.integer_depth_scale) or args.integer_depth_scale <= 0:
        raise ValueError('integer depth scale must be finite and positive')
    if not math.isfinite(args.duration) or args.duration < 0:
        raise ValueError('duration must be finite and nonnegative')
    interface = hardware_interface(args.odom_frame, args.base_frame, args.depth_frame, args.cmd_vel_topic)
    rclpy.init(args=[])
    follower, mppi, pulse = None, None, None
    nodes, executors, threads = [], [], []
    try:
        tf = TfReceiver()
        nodes.append(tf)
        follower = HardwareFollower(use_follow_intent=True, use_camera_observation=True, use_process_search=True,
                                    tf_buffer=tf.buffer, interface=interface)
        nodes.append(follower)
        inputs = HardwareInputs(args, interface, tf.buffer)
        nodes.append(inputs)
        for node in nodes:
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            executors.append(executor)
            thread = threading.Thread(target=spin_until_shutdown, args=(executor,), daemon=True)
            threads.append(thread)
            thread.start()
        pulse = ExecutorPulse(executors)
        pulse.start()
        mppi = MppiRuntime(interface=interface)
        mppi.start()
        follower.get_logger().info(f'Hardware runtime: {args.odom_topic} input, final Twist output {interface.command_topic}; automatic following from valid sensor data')
        deadline = time.monotonic() + args.duration if args.duration else math.inf
        while rclpy.ok() and time.monotonic() < deadline:
            time.sleep(.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if pulse is not None:
                pulse.close()
            for executor in executors:
                executor.shutdown(timeout_sec=5.)
            for thread in threads:
                thread.join(timeout=5.)
            if follower is not None:
                if rclpy.ok():
                    follower.cmd_pub.publish(Twist())
                follower.close_depth_worker()
                follower.core.close()
        finally:
            try:
                if mppi is not None:
                    mppi.close()
            finally:
                for node in reversed(nodes):
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == '__main__':
    main()
