"""ROS 接口：模拟传感器、独立跟随节点和浏览器观测数据。"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import copy
import csv
import json
import math
from pathlib import Path
from queue import Full, Queue
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.clock import Clock as RclClock, ClockType
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformException
from rclpy.time import Time as RosTime
from unitree_go.msg import LowState
from .controller import Pose
from .simulation import ROOT
from .camera_profile import (WIDTH, HEIGHT, FOVY, HFOV, BASELINE, CAMERA_CENTER, camera_position,
                             camera_intrinsic, mounting_geometry)
from .policy import yaw_from_quaternion
from .navigation import NavigationController
from .scenarios import SCENARIOS
from .scenario_route import route_points
from .mppi_bridge import MppiBridge
from .observation_geometry import CameraGroundModel


def timestamp(seconds):
    """把同一个 MuJoCo 时间源转换为 ROS 时间戳。"""
    nanoseconds = int(round(seconds * 1e9))
    return Time(sec=nanoseconds // 1000000000, nanosec=nanoseconds % 1000000000)


def seconds(stamp):
    """将 ROS 时间戳还原为秒。"""
    return stamp.sec + stamp.nanosec * 1e-9


def vector(message, values):
    """设置 ROS 三维向量字段。"""
    message.x, message.y, message.z = [float(value) for value in values]


class CallbackTrace:
    """保存一个回调的轻量墙钟轨迹；整份字典原子替换，不让诊断锁阻塞 ROS。"""
    def __init__(self):
        """初始化尚未进入回调的计数及阶段，时间门槛只由诊断监视器使用。"""
        self.state = dict(entry_wall=None, exit_wall=None, entries=0, exits=0,
                          active=False, stage='NOT_STARTED', stage_wall=None)

    def enter(self):
        """标记进入及累计次数，供监视器识别没有调度到回调的空档。"""
        now = time.monotonic()
        self.state = dict(self.state, entry_wall=now, entries=self.state['entries'] + 1,
                          active=True, stage='ENTER', stage_wall=now, stages_ms={})

    def mark(self, stage):
        """记录可能阻塞的调用之前所处阶段，不执行文件 I/O。"""
        now = time.monotonic()
        durations = dict(self.state.get('stages_ms', {}))
        previous = self.state['stage']
        if self.state['stage_wall'] is not None:
            durations[previous] = durations.get(previous, 0.) + (now - self.state['stage_wall']) * 1000
        self.state = dict(self.state, stage=stage, stage_wall=now, stages_ms=durations)

    def leave(self):
        """记录正常或异常退出，保留最后一个执行阶段便于事后核对。"""
        self.mark('EXIT')
        self.state = dict(self.state, exit_wall=self.state['stage_wall'], exits=self.state['exits'] + 1,
                          active=False, previous_stages_ms=dict(self.state['stages_ms']))

    def read(self):
        """返回一次原子发布的字典副本，不与回调交叉持有锁。"""
        return dict(self.state)


class Telemetry:
    """线程安全的可视化数据缓存与 CSV 记录器，数据不会反向进入控制器。"""
    def __init__(self, log_capacity=256):
        """创建界面缓存及独立有界日志队列，磁盘写入不占用传感器调度线程。"""
        self.lock = threading.RLock()
        self.file_lock = threading.Lock()
        self.log_stats_lock = threading.Lock()
        self.log_queue = Queue(maxsize=max(1, int(log_capacity)))
        self.log_enqueued = self.log_written = self.log_dropped = self.log_failed = 0
        self.log_last_write_ms = 0.0
        self.log_error = ''
        self.log_closed = False
        self.state = {}
        self.status = {'state': 'WAITING', 'reason': '正在启动', 'command': [0.0, 0.0]}
        self.history = deque(maxlen=1200)
        self.epoch = -1
        self.last_recorded = -1.0
        self.path = ROOT / 'logs' / time.strftime('follow-demo-%Y%m%d-%H%M%S.csv')
        self.file = self.path.open('w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time', 'episode_time', 'epoch', 'robot_x', 'robot_y', 'yaw',
                              'target_x', 'target_y', 'actual_vx', 'actual_wz', 'command_vx', 'command_wz',
                              'distance', 'state', 'reason', 'target_age', 'code', 'depth_age', 'plan_ms', 'obstacle_contacts'])
        self.log_thread = threading.Thread(target=self.write_log, name='go2-log', daemon=True)
        self.log_thread.start()

    def record(self, snapshot, camera_ready, error, bridge_diagnostics=None, render_diagnostics=None):
        """更新界面与内存历史，非阻塞投递 CSV 行；慢磁盘不能拖住心跳或速度接收。"""
        csv_row = None
        with self.lock:
            if snapshot['epoch'] != self.epoch:
                self.history.clear()
                self.epoch = snapshot['epoch']
            distance = float(np.linalg.norm(snapshot['target'] - [snapshot['x'], snapshot['y']]))
            state = {key: snapshot[key] for key in ('t', 'episode_t', 'epoch', 'x', 'y', 'z', 'yaw', 'vx', 'vy', 'wz',
                                                    'ready', 'enabled', 'emergency', 'signal', 'target_mode', 'watchdog', 'rtf', 'applied',
                                                    'scenario', 'depth_enabled', 'obstacle_contacts')}
            state.update(target=snapshot['target'].tolist(), target_velocity=snapshot['target_velocity'].tolist(),
                         target_route=snapshot.get('target_route'),
                         distance=distance, control=dict(self.status), camera_ready=camera_ready, error=error,
                         odometry_source='仿真真值', obstacle_avoidance=True, log_file=self.path.name,
                         scenarios={name: value['name'] for name, value in SCENARIOS.items()},
                         scenario_description=SCENARIOS[snapshot['scenario']].get('description', '基础回归场景'),
                         scenario_route_points=route_points(SCENARIOS[snapshot['scenario']]),
                         sensor_bridge=bridge_diagnostics or {},
                         render_diagnostics=render_diagnostics or {},
                         camera_model='D435i 红外双目几何近似', camera_hfov=HFOV)
            self.state = state
            if snapshot['t'] - self.last_recorded >= 0.099:
                self.last_recorded = snapshot['t']
                command = self.status.get('command', [0, 0])
                row = dict(t=snapshot['t'], x=snapshot['x'], y=snapshot['y'], tx=float(snapshot['target'][0]),
                           ty=float(snapshot['target'][1]), v=snapshot['vx'], w=snapshot['wz'],
                           cv=command[0], cw=command[1], tv=float(np.linalg.norm(snapshot['target_velocity'])), distance=distance,
                           state=self.status.get('state'))
                self.history.append(row)
                csv_row = [snapshot['t'], snapshot['episode_t'], snapshot['epoch'], snapshot['x'], snapshot['y'], snapshot['yaw'],
                                      *snapshot['target'], snapshot['vx'], snapshot['wz'], *command, distance,
                                      self.status.get('state'), self.status.get('reason'), self.status.get('target_age'),
                                      self.status.get('navigation', {}).get('code'),
                                      self.status.get('navigation', {}).get('map', {}).get('depth_age'),
                                      self.status.get('navigation', {}).get('plan_ms'), snapshot['obstacle_contacts']]
        if csv_row is not None:
            with self.log_stats_lock:
                if self.log_closed:
                    self.log_dropped += 1
                    return
                try:
                    self.log_queue.put_nowait(csv_row)
                    self.log_enqueued += 1
                except Full:
                    # 丢弃日志而非阻塞控制；界面历史仍保留新状态，丢失数量明确可见。
                    self.log_dropped += 1

    def write_log(self):
        """唯一写文件线程按 FIFO 消费日志；文件锁不会与界面缓存锁交叉持有。"""
        try:
            while True:
                row = self.log_queue.get()
                try:
                    if row is None:
                        break
                    started = time.monotonic()
                    try:
                        with self.file_lock:
                            self.writer.writerow(row)
                            self.file.flush()
                    except Exception as error:
                        with self.log_stats_lock:
                            self.log_failed += 1
                            self.log_error = str(error)
                    else:
                        with self.log_stats_lock:
                            self.log_written += 1
                    finally:
                        with self.log_stats_lock:
                            self.log_last_write_ms = (time.monotonic() - started) * 1000
                finally:
                    self.log_queue.task_done()
        finally:
            with self.file_lock:
                self.file.close()

    def log_diagnostics(self):
        """返回日志队列/写入诊断，读取过程不等待磁盘。"""
        with self.log_stats_lock:
            return dict(capacity=self.log_queue.maxsize, queued=self.log_queue.qsize(),
                        enqueued=self.log_enqueued, written=self.log_written, dropped=self.log_dropped,
                        failed=self.log_failed, last_write_ms=self.log_last_write_ms,
                        error=self.log_error, closed=self.log_closed, worker_alive=self.log_thread.is_alive())

    def download_log(self):
        """按独立文件锁导出已写入的 CSV 快照，不占用传感器使用的缓存锁。"""
        with self.file_lock:
            if not self.file.closed:
                self.file.flush()
            return self.path.read_bytes()

    def read(self):
        """返回可 JSON 编码的页面状态副本。"""
        with self.lock:
            payload = {**self.state, 'history': list(self.history)}
        payload['logging'] = self.log_diagnostics()
        return payload

    def close(self):
        """ROS 线程停止后禁止再入队，排空已有日志并等待唯一写线程关闭文件。"""
        with self.log_stats_lock:
            if self.log_closed:
                return
            self.log_closed = True
        # 仅退出流程允许等队列；record 的 put_nowait 始终不等待日志线程。
        self.log_queue.put(None)
        self.log_thread.join()


class SensorBridge(Node):
    """将物理状态变成 ROS 输入，并把唯一的速度命令话题送回模拟器。"""
    def __init__(self, simulation, telemetry):
        """建立发布者；以单调时钟定周期，持续提供仿真 /clock 和输入心跳。"""
        super().__init__('go2_follow_simulator', parameter_overrides=[Parameter('use_sim_time', value=False)])
        self.simulation, self.telemetry = simulation, telemetry
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        # Humble controller_server 的里程计订阅要求可靠传输；可靠发布仍兼容最佳努力订阅。
        # 不匹配会使 MPPI 收不到实测速度，错误地从零速度预测下一段运动。
        self.odom_pub = self.create_publisher(Odometry, '/follow_demo/odom', 5)
        self.uwb_pub = self.create_publisher(PointStamped, '/uwb/target_point', qos_profile_sensor_data)
        self.operator_pub = self.create_publisher(String, '/follow_demo/operator', 10)
        self.low_pub = self.create_publisher(LowState, '/lowstate', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, '/camera/imu', qos_profile_sensor_data)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
        self.image_pubs = {side: self.create_publisher(Image, f'/camera/{side}/image_raw', qos_profile_sensor_data) for side in ('left', 'right')}
        self.info_pubs = {side: self.create_publisher(CameraInfo, f'/camera/{side}/camera_info', qos_profile_sensor_data) for side in ('left', 'right')}
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_rect_raw', qos_profile_sensor_data)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.create_subscription(String, '/follow_demo/control_status', self.on_status, 10)
        self.rng = np.random.default_rng(7)
        self.pending = deque()
        self.last_uwb = -1.0
        self.last_state = -1.0
        self.last_frame = -1.0
        self.last_depth = -1.0
        self.epoch = -1
        self.last_publish_ms = self.max_publish_ms = 0.0
        self.callback_trace = CallbackTrace()
        # 系统日期在 WSL 校时时可回跳数秒；周期任务不能等待旧日期重新到来。
        # 这里只规定调度时钟，所有消息 stamp 仍取同一物理仿真时间源。
        self.publish_clock = RclClock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.01, self.publish_snapshot, clock=self.publish_clock)

    def on_command(self, message):
        """把 ROS 速度送到执行器，看门狗独立于跟随节点运行。"""
        self.simulation.set_command(message.linear.x, message.linear.y, message.angular.z)

    def on_status(self, message):
        """收集控制器诊断，不从诊断中推导或改变运动命令。"""
        with self.telemetry.lock:
            self.telemetry.status = json.loads(message.data)

    def publish_snapshot(self):
        """记录上次/峰值发布耗时，定位本地调度阻塞，不改变任何健康门槛。"""
        started = time.monotonic()
        self.callback_trace.enter()
        try:
            self.publish_snapshot_body()
        finally:
            self.callback_trace.leave()
            self.last_publish_ms = (time.monotonic() - started) * 1000
            self.max_publish_ms = max(self.max_publish_ms, self.last_publish_ms)

    def publish_snapshot_body(self):
        """将同一个物理快照发布为时钟、传感器和界面状态。"""
        self.callback_trace.mark('SIM_SNAPSHOT')
        with self.simulation.lock:
            state = self.simulation.snapshot.copy()
            frames = self.simulation.frames
            depth_frame = self.simulation.depth_frame
            render_diagnostics = copy.deepcopy(self.simulation.render_diagnostics)
        if not state or state['t'] <= self.last_state:
            return
        self.last_state = state['t']
        if self.epoch != state['epoch']:
            self.epoch = state['epoch']
            self.pending.clear()
        stamp = timestamp(state['t'])
        self.callback_trace.mark('CLOCK_PUBLISH')
        self.clock_pub.publish(Clock(clock=stamp))
        operator = {key: state[key] for key in ('epoch', 'ready', 'enabled', 'emergency', 'signal', 'clearance_confirmed')}
        self.callback_trace.mark('OPERATOR_PUBLISH')
        self.operator_pub.publish(String(data=json.dumps(operator)))
        odometry = Odometry()
        odometry.header.stamp, odometry.header.frame_id, odometry.child_frame_id = stamp, 'odom', 'base_footprint'
        vector(odometry.pose.pose.position, [state['x'], state['y'], 0])
        odometry.pose.pose.orientation.w, odometry.pose.pose.orientation.z = math.cos(state['yaw'] / 2), math.sin(state['yaw'] / 2)
        vector(odometry.twist.twist.linear, [state['vx'], state['vy'], 0])
        odometry.twist.twist.angular.z = state['wz']
        self.callback_trace.mark('ODOM_PUBLISH')
        self.odom_pub.publish(odometry)
        self.callback_trace.mark('TF_PUBLISH')
        self.publish_tf(state, stamp)
        self.callback_trace.mark('LOWSTATE_PUBLISH')
        self.publish_lowstate(state, stamp)
        self.callback_trace.mark('UWB_PUBLISH')
        self.publish_uwb(state)
        if frames and frames[0] > self.last_frame:
            self.last_frame = frames[0]
            for side, frame in zip(('left', 'right'), frames[1:]):
                self.callback_trace.mark('STEREO_' + side.upper())
                self.publish_image(side, frame, timestamp(frames[0]))
        if depth_frame and state['depth_enabled'] and depth_frame[0] > self.last_depth:
            self.last_depth = depth_frame[0]
            self.callback_trace.mark('DEPTH_PUBLISH')
            self.publish_depth(depth_frame[1], timestamp(depth_frame[0]))
        self.callback_trace.mark('TELEMETRY_CACHE')
        self.telemetry.record(state, bool(frames), self.simulation.error or self.simulation.render_error,
                              dict(previous_callback_ms=self.last_publish_ms, maximum_callback_ms=self.max_publish_ms),
                              render_diagnostics)

    def publish_tf(self, state, stamp):
        """广播 odom→base_link 真值 TF 与相机外参；首版不与 CAPO TF 混用。"""
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id, transform.child_frame_id = stamp, 'odom', 'base_footprint'
        vector(transform.transform.translation, [state['x'], state['y'], 0])
        rotation = transform.transform.rotation
        rotation.w, rotation.z = math.cos(state['yaw'] / 2), math.sin(state['yaw'] / 2)
        transforms = [transform]
        body = TransformStamped()
        body.header.stamp, body.header.frame_id, body.child_frame_id = stamp, 'base_footprint', 'base_link'
        body.transform.translation.z = state['z']
        w, x, y, z = [float(value) for value in state['quaternion']]
        cosine, sine = math.cos(state['yaw'] / 2), math.sin(state['yaw'] / 2)
        body.transform.rotation.w, body.transform.rotation.x = cosine * w + sine * z, cosine * x + sine * y
        body.transform.rotation.y, body.transform.rotation.z = cosine * y - sine * x, cosine * z - sine * w
        transforms.append(body)
        _, optical, imu = mounting_geometry(self.simulation.camera_pitch_deg)
        for child, position, quaternion in [
            ('camera_left_optical', camera_position('left'), optical),
            ('camera_right_optical', camera_position('right'), optical),
            ('camera_imu', CAMERA_CENTER, imu),
        ]:
            item = TransformStamped()
            item.header.stamp, item.header.frame_id, item.child_frame_id = stamp, 'base_link', child
            vector(item.transform.translation, position)
            item.transform.rotation.x, item.transform.rotation.y, item.transform.rotation.z, item.transform.rotation.w = [float(value) for value in quaternion]
            transforms.append(item)
        self.tf.sendTransform(transforms)

    def publish_lowstate(self, state, stamp):
        """发布关节/机身状态以及相机处的原始 IMU；相机 IMU 不填融合姿态。"""
        low = LowState()
        low.imu_state.quaternion = [float(value) for value in state['quaternion']]
        low.imu_state.gyroscope = [float(value) for value in state['gyro']]
        low.imu_state.accelerometer = [float(value) for value in state['accel']]
        for index, (position, velocity) in enumerate(zip(state['q'], state['dq'])):
            low.motor_state[index].q, low.motor_state[index].dq = float(position), float(velocity)
        low.foot_force = [int(min(32767, max(0, value))) for value in state['foot_force']]
        self.low_pub.publish(low)
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = [name + '_joint' for name in self.simulation.policy.names]
        joints.position, joints.velocity = state['q'].tolist(), state['dq'].tolist()
        self.joint_pub.publish(joints)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'camera_imu'
        imu.orientation_covariance[0] = -1.0
        vector(imu.angular_velocity, state['camera_gyro'])
        vector(imu.linear_acceleration, state['camera_acc'])
        self.imu_pub.publish(imu)

    def publish_uwb(self, state):
        """生成带采集时间的 UWB 平面点，加入测距/测角噪声和 60ms 传输延迟。"""
        if not state['signal']:
            self.pending.clear()
            return
        if state['t'] - self.last_uwb >= 0.049:
            self.last_uwb = state['t']
            dx, dy = state['target'] - [state['x'], state['y']]
            distance = max(0, math.hypot(dx, dy) + self.rng.normal(0, 0.015))
            bearing = math.atan2(dy, dx) - state['yaw'] + self.rng.normal(0, 0.003)
            point = PointStamped()
            point.header.stamp, point.header.frame_id = timestamp(state['t']), 'base_footprint'
            point.point.x, point.point.y = distance * math.cos(bearing), distance * math.sin(bearing)
            self.pending.append((state['t'] + 0.06, point))
        while self.pending and self.pending[0][0] <= state['t']:
            self.uwb_pub.publish(self.pending.popleft()[1])

    def publish_image(self, side, frame, stamp):
        """发布同步灰度双目和理想针孔内参；D435i 的实测内参与安装外参仍待接入。"""
        image = Image()
        image.header.stamp, image.header.frame_id = stamp, f'camera_{side}_optical'
        image.width, image.height, image.encoding, image.step = WIDTH, HEIGHT, 'mono8', WIDTH
        image.data = frame.tobytes()
        self.image_pubs[side].publish(image)
        focal, _, cx, cy = camera_intrinsic(WIDTH, HEIGHT)
        info = CameraInfo()
        info.header = image.header
        info.width, info.height, info.distortion_model = WIDTH, HEIGHT, 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, cx, -focal * BASELINE if side == 'right' else 0.0,
                  0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pubs[side].publish(info)

    def publish_depth(self, frame, stamp):
        """发布左光心的米制前视深度和匹配内参，缩小分辨率以控制 WSL 图像负载。"""
        image = Image()
        image.header.stamp, image.header.frame_id = stamp, 'camera_left_optical'
        image.height, image.width = frame.shape
        image.encoding, image.step = '32FC1', image.width * 4
        image.data = frame.astype('<f4').tobytes()
        self.depth_pub.publish(image)
        focal, _, cx, cy = camera_intrinsic(image.width, image.height)
        info = CameraInfo()
        info.header, info.height, info.width = image.header, image.height, image.width
        info.distortion_model, info.d = 'plumb_bob', [0.0] * 5
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.depth_info_pub.publish(info)


class TfReceiver(Node):
    """独立接收采集时间 TF，避免控制计算占满调度器而令新深度无法找到外参。"""
    def __init__(self):
        """只维护线程安全 TF 缓冲区，不接触控制状态或更改变换时间。"""
        super().__init__('go2_follow_tf_receiver', parameter_overrides=[Parameter('use_sim_time', value=False)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)


class FollowerNode(Node):
    """独立控制节点，通过 ROS 深度、TF、UWB 和里程计规划，不访问场景几何。"""
    def __init__(self, use_follow_intent=False, use_camera_observation=False, tf_buffer=None, use_process_search=False):
        """建立可替换为实机传感器的标准 ROS 输入和速度输出接口。"""
        super().__init__('go2_local_follower', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.use_follow_intent = use_follow_intent
        self.use_camera_observation = use_camera_observation
        self.use_process_search = use_process_search
        self.core = NavigationController(use_follow_intent=use_follow_intent,
                                         use_camera_observation=use_camera_observation,
                                         use_process_search=use_process_search)
        self.operator = dict(enabled=False, emergency=False, signal=False, ready=False, epoch=-1)
        self.last_operator_wall = 0.0
        self.tf_buffer = Buffer() if tf_buffer is None else tf_buffer
        self.tf_listener = TransformListener(self.tf_buffer, self) if tf_buffer is None else None
        self.pending_depth = None
        self.depth_info = None
        # 唯一后台任务只修改地图副本；ROS 控制线程仍独占 core 与已发布地图。
        # pending_depth 始终是最新一帧，不向执行器堆积图像任务。
        self.depth_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='follow-depth')
        self.depth_future = None
        self.depth_job = None
        self.depth_worker_closed = False
        self.depth_worker_ms = 0.0
        self.depth_applied = 0
        self.depth_rejected = 0
        self.depth_rebased = 0
        self.depth_rejected_reason = ''
        self.depth_worker_error = ''
        self.depth_submit_reason = 'NOT_RECEIVED'
        self.depth_tf_waits = 0
        self.depth_tf_error = ''
        self.epoch_start = 0.0
        self.last_status = -1.0
        self.last_control_ms = self.max_control_ms = 0.0
        self.callback_trace = CallbackTrace()
        self.mppi = MppiBridge(self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/follow_demo/control_status', 10)
        # 里程计用于当前执行保护：旧消息不能在控制负载升高时排队冒充最新状态。
        latest_sensor_qos = copy.copy(qos_profile_sensor_data)
        latest_sensor_qos.depth = 1
        self.create_subscription(Odometry, '/follow_demo/odom', self.on_odometry, latest_sensor_qos)
        self.create_subscription(PointStamped, '/uwb/target_point', self.on_target, qos_profile_sensor_data)
        self.create_subscription(String, '/follow_demo/operator', self.on_operator, QoSProfile(depth=1))
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.on_depth_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.on_depth, qos_profile_sensor_data)
        self.timer = self.create_timer(0.02, self.control)

    def on_odometry(self, message):
        """记录用于 UWB 采样时间对齐的里程计历史。"""
        position, orientation = message.pose.pose.position, message.pose.pose.orientation
        self.core.history.add(Pose(seconds(message.header.stamp), position.x, position.y,
                                   yaw_from_quaternion([orientation.w, orientation.x, orientation.y, orientation.z]),
                                   (message.twist.twist.linear.x,message.twist.twist.linear.y,message.twist.twist.angular.z)))
        self.core.motion = (message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.angular.z)

    def on_depth_info(self, message):
        """缓存与深度光心一致的针孔内参，错误坐标系不参与投影。"""
        if message.header.frame_id == 'camera_left_optical':
            self.depth_info = message

    def on_depth(self, message):
        """仅保留最新深度，避免处理积压图像导致地图落后实际运动。"""
        if message.header.frame_id == 'camera_left_optical' and seconds(message.header.stamp) >= self.epoch_start:
            if self.pending_depth is None or seconds(message.header.stamp) > seconds(self.pending_depth.header.stamp):
                self.pending_depth = message

    @staticmethod
    def fuse_depth(snapshot, depth, intrinsic, rotation, translation, stamp):
        """后台仅融合深拷贝地图并返回结果，不访问 ROS、实时 core 或当前机身姿态。"""
        started = time.monotonic()
        integrated = snapshot.integrate(depth, intrinsic, rotation, translation, stamp)
        return snapshot, integrated, (time.monotonic() - started) * 1000

    def close_depth_worker(self):
        """在 ROS 调度线程结束后关闭唯一融合线程，不遗留后台地图任务。"""
        if self.depth_worker_closed:
            return
        self.depth_worker_closed = True
        self.depth_pool.shutdown(wait=True, cancel_futures=True)
        self.depth_future, self.depth_job, self.pending_depth = None, None, None

    def accept_depth_result(self, now):
        """在控制线程事务接入地图；重置、窗口移动、确认净空或过期帧均使旧副本失效。"""
        if self.depth_future is None or not self.depth_future.done():
            return
        future, job = self.depth_future, self.depth_job
        self.depth_future, self.depth_job = None, None
        try:
            snapshot, integrated, self.depth_worker_ms = future.result()
        except Exception as error:
            # 保留上一张证据地图，健康时间不刷新，现有 MAP_STALE 会按原门槛停车。
            self.depth_worker_error = str(error)
            self.get_logger().error(f'深度地图后台融合失败：{error}')
            if self.pending_depth is not None and seconds(self.pending_depth.header.stamp) <= job['stamp']:
                self.pending_depth = None
            return
        reason = ''
        if job['epoch'] != self.operator['epoch'] or job['grid'] is not self.core.grid:
            reason = 'DEPTH_EPOCH_CHANGED'
        elif job['version'] != self.core.grid.version:
            if job.get('evidence_version') != self.core.grid.evidence_version:
                # 人工确认或其他观测已改内容，不能用旧副本覆盖真实新证据。
                reason = 'DEPTH_MAP_CHANGED'
            else:
                # 仅跨过一个滚动格时，证据的世界坐标没有改变；重对齐布局后接入。
                # 新露出的窗口边缘仍未知，不能用插值填空或把未观测区变为自由。
                grid = self.core.grid
                center = grid.origin + (grid.size // 2 + .5) * grid.resolution
                snapshot.recenter(*center)
                self.depth_rebased = getattr(self, 'depth_rebased', 0) + 1
        if not reason and (not self.epoch_start <= job['stamp'] <= now or now - job['stamp'] > self.core.depth_timeout):
            reason = 'DEPTH_FRAME_STALE'
        elif not reason and self.core.grid.last_depth is not None and job['stamp'] <= self.core.grid.last_depth:
            reason = 'DEPTH_FRAME_SUPERSEDED'
        elif not reason and not integrated:
            reason = 'DEPTH_FRAME_INVALID'
        if reason:
            self.depth_rejected += 1
            self.depth_rejected_reason = reason
        else:
            # 所有源状态仍一致才一次性换图；工作线程从未获得这张可写实时地图。
            self.core.grid = snapshot
            self.depth_applied += 1
            self.depth_worker_error = ''
        if reason != 'DEPTH_MAP_CHANGED' and self.pending_depth is not None:
            if seconds(self.pending_depth.header.stamp) <= job['stamp']:
                self.pending_depth = None

    def integrate_depth(self, now):
        """接入完成副本并提交最新深度；融合运算不再占用控制/心跳回调。"""
        self.accept_depth_result(now)
        if self.depth_worker_closed or self.depth_future is not None:
            self.depth_submit_reason = 'CLOSED' if self.depth_worker_closed else 'WORKER_BUSY'
            return
        message, info = self.pending_depth, self.depth_info
        if message is None or info is None:
            self.depth_submit_reason = 'NO_FRAME' if message is None else 'NO_CAMERA_INFO'
            return
        stamp = seconds(message.header.stamp)
        if (stamp < self.epoch_start or now - stamp > self.core.depth_timeout
                or (self.core.grid.last_depth is not None and stamp <= self.core.grid.last_depth)):
            self.pending_depth = None
            self.depth_submit_reason = 'FRAME_EXPIRED_OR_APPLIED'
            return
        if stamp > now or (message.width, message.height) != (info.width, info.height):
            self.depth_submit_reason = 'WAIT_CLOCK' if stamp > now else 'CAMERA_INFO_SIZE'
            return
        try:
            transform = self.tf_buffer.lookup_transform('odom', message.header.frame_id, RosTime.from_msg(message.header.stamp)).transform
        except TransformException as error:
            self.depth_submit_reason = 'WAIT_CAPTURE_TF'
            self.depth_tf_waits = getattr(self, 'depth_tf_waits', 0) + 1
            self.depth_tf_error = str(error)
            return
        if message.encoding != '32FC1' or message.step < message.width * 4 or len(message.data) != message.height * message.step:
            self.pending_depth = None
            self.depth_submit_reason = 'FRAME_ENCODING_INVALID'
            return
        dtype = '>f4' if message.is_bigendian else '<f4'
        depth = np.frombuffer(message.data, dtype=dtype).reshape(message.height, message.step // 4)[:, :message.width]
        q = transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                             [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                             [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        translation = np.array([transform.translation.x, transform.translation.y, transform.translation.z])
        if self.core.history.values:
            pose = self.core.history.values[-1]
            self.core.grid.recenter(pose.x, pose.y)
        grid = self.core.grid
        snapshot = copy.deepcopy(grid)
        # 用采集时刻的机身姿态去掉 odom 平移/航向，保留 TF 中的真实相机高度和倾斜。
        # 模型只随同一帧地图副本事务接入；旧 epoch、旧窗口或过期帧不能覆盖当前标定预测。
        captured_pose = self.core.history.at(stamp) if hasattr(self.core.history, 'at') else None
        try:
            snapshot.camera_model = (None if captured_pose is None else CameraGroundModel.from_capture(
                [info.k[0], info.k[4], info.k[2], info.k[5]], (message.height, message.width),
                rotation, translation, (captured_pose.x, captured_pose.y), captured_pose.yaw))
        except ValueError as error:
            self.depth_worker_error = str(error)
            self.pending_depth = None
            self.depth_submit_reason = 'CAMERA_MODEL_INVALID'
            return
        self.depth_job = dict(epoch=self.operator['epoch'], grid=grid, version=grid.version,
                              evidence_version=grid.evidence_version,
                              stamp=stamp, submitted_wall=time.monotonic())
        self.depth_future = self.depth_pool.submit(self.fuse_depth, snapshot, depth.copy(),
                                                   [info.k[0], info.k[4], info.k[2], info.k[5]],
                                                   rotation, translation, stamp)
        self.depth_submit_reason = 'SUBMITTED'
        self.depth_tf_error = ''
        # 保留这一最新帧：若移动窗口使副本失效且尚无更新帧，仍可在新地图副本重算。
        # on_depth 只用更新的采集时间替换它，任务完成前不会再提交第二个任务。

    def on_target(self, message):
        """只接受约定坐标系的目标点，避免错误外参静默混入控制。"""
        if message.header.frame_id == 'base_footprint':
            self.core.observe(message.point.x, message.point.y, seconds(message.header.stamp))

    def on_operator(self, message):
        """更新操作状态；场景重置时重建估计器，避免上一轮速度估计残留。"""
        operator = json.loads(message.data)
        if operator['epoch'] != self.operator['epoch']:
            self.mppi.reset()
            self.core.close()
            # 重置场景时保留启动选定的规划语义，不能无声退回另一实验模式。
            self.core = NavigationController(use_follow_intent=self.use_follow_intent,
                                             use_camera_observation=self.use_camera_observation,
                                             use_process_search=self.use_process_search)
            self.pending_depth = None
            self.epoch_start = self.get_clock().now().nanoseconds * 1e-9
            # 正在执行的旧任务不能强停，保留单一在途占位，返回后按 epoch / 地图身份拒收。
            if self.depth_future is not None and self.depth_future.cancel():
                self.depth_future, self.depth_job = None, None
        if operator.get('clearance_confirmed') and not self.operator.get('clearance_confirmed'):
            self.core.confirmation_requested = True
        self.operator = operator
        self.last_operator_wall = time.monotonic()

    def control(self):
        """测量控制回调墙钟耗时，耗时诊断不参与指令或失效判定。"""
        started = time.monotonic()
        self.callback_trace.enter()
        try:
            self.control_body()
        finally:
            self.callback_trace.leave()
            self.last_control_ms = (time.monotonic() - started) * 1000
            self.max_control_ms = max(self.max_control_ms, self.last_control_ms)

    def control_body(self):
        """发布经过同一控制核心限制的最终命令及相应诊断。"""
        self.callback_trace.mark('GET_CLOCK')
        now = self.get_clock().now().nanoseconds * 1e-9
        operator = self.operator
        self.callback_trace.mark('DEPTH_RESULT')
        self.integrate_depth(now)
        if self.core.confirmation_requested and not operator['enabled'] and operator['ready']:
            self.core.confirm_start(now)
            self.core.confirmation_requested = False
        self.callback_trace.mark('CORE_STEP')
        if time.monotonic() - self.last_operator_wall > 0.35:
            # 状态通信中断也通过统一撤销入口，恢复后不能重放中断前的观察动作。
            command = self.core.halt('INPUT_LOST','OPERATOR_STALE','仿真状态通信中断',now=now)
        else:
            command = self.core.step(now, operator['enabled'], operator['signal'], operator['ready'], operator['emergency'])
        self.callback_trace.mark('MPPI_UPDATE')
        self.mppi.update(now)
        message = Twist()
        message.linear.x, message.angular.z = command
        self.callback_trace.mark('COMMAND_PUBLISH')
        self.cmd_pub.publish(message)
        if now - self.last_status < 0.2:
            return
        self.last_status = now
        self.callback_trace.mark('STATUS_BUILD')
        status = dict(control_t=now, state=self.core.state, reason=self.core.reason, command=list(command),
                      control_callback=dict(previous_ms=self.last_control_ms, maximum_ms=self.max_control_ms,
                                            previous_stages_ms=self.callback_trace.read().get('previous_stages_ms', {})),
                      estimated_target=self.core.target, estimated_target_velocity=self.core.target_velocity,
                      target_age=None if self.core.target_stamp is None else max(0, now - self.core.target_stamp),
                      accepted=self.core.accepted, rejected=self.core.rejected, navigation=self.core.diagnostics(now),
                      depth_fusion=dict(busy=self.depth_future is not None, applied=self.depth_applied,
                                        rebased=self.depth_rebased,
                                        rejected=self.depth_rejected, last_rejected=self.depth_rejected_reason,
                                        last_worker_ms=self.depth_worker_ms, error=self.depth_worker_error,
                                        submit_reason=self.depth_submit_reason, tf_waits=self.depth_tf_waits,
                                        tf_error=self.depth_tf_error,
                                        pending_stamp=None if self.pending_depth is None else seconds(self.pending_depth.header.stamp)),
                      mppi_error=self.mppi.error,mppi_action=self.mppi.diagnostics())
        self.callback_trace.mark('STATUS_PUBLISH')
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
