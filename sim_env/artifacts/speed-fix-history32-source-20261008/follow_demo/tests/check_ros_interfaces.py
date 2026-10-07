"""独立 ROS 进程检查演示输出，验证跨进程通信和双目时间戳契约。"""
from collections import defaultdict
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu
from unitree_go.msg import LowState


def stamp_key(stamp):
    """保留完整纳秒精度，避免浮点比较掩盖双目时间偏差。"""
    return stamp.sec * 1000000000 + stamp.nanosec


class Observer(Node):
    """只订阅公开话题，不导入或访问仿真器内部对象。"""
    def __init__(self):
        """建立独立观察者并收集足够的消息样本。"""
        super().__init__('follow_demo_interface_check')
        self.counts = defaultdict(int)
        self.stamps = defaultdict(set)
        self.last_stamp = {}
        self.errors = []
        self.info = {}
        self.accelerations = []
        self.bindings = []
        for name, kind, topic in [
            ('clock', Clock, '/clock'), ('command', Twist, '/cmd_vel'),
            ('odom', Odometry, '/follow_demo/odom'), ('imu', Imu, '/camera/imu'),
            ('lowstate', LowState, '/lowstate'),
            ('left', Image, '/camera/left/image_raw'), ('right', Image, '/camera/right/image_raw'),
            ('left_info', CameraInfo, '/camera/left/camera_info'),
            ('right_info', CameraInfo, '/camera/right/camera_info'),
            ('depth', Image, '/camera/depth/image_rect_raw'), ('depth_info', CameraInfo, '/camera/depth/camera_info'),
        ]:
            self.bindings.append(self.create_subscription(kind, topic,
                lambda message, key=name: self.observe(key, message), qos_profile_sensor_data))

    def observe(self, name, message):
        """检查消息格式、有限数值和每个输入流的时间单调性。"""
        self.counts[name] += 1
        if name == 'clock' or hasattr(message, 'header'):
            stamp = stamp_key(message.clock if name == 'clock' else message.header.stamp)
            self.stamps[name].add(stamp)
            if stamp < self.last_stamp.get(name, 0):
                self.errors.append(name + ' 时间倒退')
            self.last_stamp[name] = stamp
        if name in ('left', 'right'):
            if (message.width, message.height, message.encoding, len(message.data)) != (424, 240, 'mono8', 424 * 240):
                self.errors.append(name + ' 图像格式不匹配')
        elif name == 'depth':
            if (message.width, message.height, message.encoding, len(message.data)) != (212, 120, '32FC1', 212 * 120 * 4):
                self.errors.append('深度图像格式不匹配')
            else:
                depth = np.frombuffer(message.data, dtype='<f4')
                valid = depth[np.isfinite(depth)]
                if not len(valid) or np.min(valid) < 0.25 or np.max(valid) > 5.0:
                    self.errors.append('米制深度量程不匹配')
        elif name.endswith('_info'):
            self.info[name] = message
        elif name == 'imu':
            values = [message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z,
                      message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z]
            if not all(math.isfinite(value) for value in values) or message.orientation_covariance[0] != -1.0:
                self.errors.append('IMU 非有限值或未标记原始测量')
            self.accelerations.append(math.sqrt(sum(value * value for value in values[3:])))
        elif name == 'odom':
            pose = message.pose.pose.position
            if not all(math.isfinite(value) for value in (pose.x, pose.y, message.twist.twist.linear.x)):
                self.errors.append('里程计非有限值')
        elif name == 'command':
            if not all(math.isfinite(value) for value in (message.linear.x, message.angular.z)):
                self.errors.append('命令非有限值')
        elif name == 'lowstate':
            if not all(math.isfinite(motor.q) and math.isfinite(motor.dq) for motor in message.motor_state[:12]):
                self.errors.append('关节非有限值')


def main():
    """观察十秒后输出可保存的检查结果，不驱动机器人或目标。"""
    rclpy.init()
    observer = Observer()
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.05)
        pairs = observer.stamps['left'] & observer.stamps['right']
        baseline = None
        if 'right_info' in observer.info:
            info = observer.info['right_info']
            baseline = float(-info.p[3] / info.p[0]) if info.p[0] else None
        counts = dict(observer.counts)
        checks = {
            'all_topics_received': all(counts.get(name, 0) >= 5 for name in
                ('clock', 'command', 'odom', 'imu', 'lowstate', 'left', 'right', 'left_info', 'right_info', 'depth', 'depth_info')),
            'stereo_identical_capture_stamp': len(pairs) >= 5 and len(pairs) >= min(counts.get('left', 0), counts.get('right', 0)) - 1,
            'images_share_imu_clock': len(pairs & observer.stamps['imu']) >= len(pairs) - 1,
            'd435i_nominal_simulation_baseline': baseline is not None and abs(baseline - 0.050) < 1e-8,
            'depth_shares_stereo_capture_stamp': len(observer.stamps['depth'] & pairs) >= 5,
            'depth_has_matching_intrinsic_stamp': len(observer.stamps['depth'] & observer.stamps['depth_info']) >= 5,
            'finite_and_monotonic': not observer.errors,
        }
        report = dict(passed=all(checks.values()), checks=checks, counts=counts, stereo_pairs=len(pairs),
                      camera_profile='D435i 红外双目几何近似', image_format='424x240 mono8',
                      baseline_m=baseline, errors=observer.errors[:20],
                      imu_acceleration_norm_mean=sum(observer.accelerations) / max(1, len(observer.accelerations)))
        destination = Path.home() / 'go2_sim/artifacts/navigation-ros-check.json'
        destination.write_text(json.dumps(report, indent=2, ensure_ascii=False))
        print(json.dumps(report, indent=2, ensure_ascii=False))
    finally:
        observer.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
