"""用理想积分底盘和全自由地图隔离 MPPI 速度选择；只做诊断，不修改生产参数。"""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import FollowPath
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster

from follow_demo import mppi_runtime
from std_msgs.msg import Float64MultiArray


PROFILES = {
    'short': dict(reference_m=1.5, overrides={}),
    'long': dict(reference_m=3.5, overrides={}),
    'matched_dt': dict(reference_m=3.5, overrides=dict(model_dt=1 / 30, time_steps=72)),
    'smaller_noise': dict(reference_m=3.5, overrides=dict(vx_std=.10)),
    'no_effort_cost': dict(reference_m=3.5, overrides=dict(gamma=0.)),
    'short_no_effort_cost': dict(reference_m=1.5, overrides=dict(gamma=0.)),
}


class SpeedRig(Node):
    """只发布诊断地图、位姿和滚动直线路径，不接入相机、UWB或生产执行保护。"""
    def __init__(self, reference_m, name):
        """建立与标准控制器相同的消息接口，真实 ROS 回调仍决定是否收到候选命令。"""
        super().__init__(f'go2_mppi_speed_diagnostic_{name}')
        self.reference_m = reference_m
        self.t = self.x = self.y = self.yaw = 0.
        self.raw = [0., 0.]
        self.actual = [0., 0.]
        self.command_t = -math.inf
        self.goal_handle = self.pending = None
        self.sequence = 0
        self.first_accepted = None
        self.results = []
        self.rows = []
        self.last_map = self.last_sent = self.last_sample = -math.inf
        self.lifecycle_state = None
        self.lifecycle_pending = None
        self.last_lifecycle_query = -math.inf
        self.rejections = 0
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.clock_pub = self.create_publisher(Clock, '/clock', 1)
        self.map_pub = self.create_publisher(OccupancyGrid, '/follow_demo/local_geometry', qos)
        self.odom_pub = self.create_publisher(Odometry, '/follow_demo/odom', 1)
        self.speed_pub = self.create_publisher(TwistStamped, '/follow_demo/reference_speed', 1)
        self.braking_pub = self.create_publisher(Float64MultiArray, '/follow_demo/braking_limits', 1)
        self.tf = TransformBroadcaster(self)
        self.client = ActionClient(self, FollowPath, '/go2_follow_mppi/follow_path')
        self.lifecycle_client = self.create_client(GetState, '/go2_follow_mppi/controller_server/get_state')
        self.subscription = self.create_subscription(Twist, '/follow_demo/mppi_cmd', self.on_command, 1)

    def stamp(self):
        """所有诊断消息共用同一个人工仿真时刻，避免混入系统时间。"""
        nanoseconds = round(self.t * 1e9)
        return Time(sec=nanoseconds // 1000000000, nanosec=nanoseconds % 1000000000)

    def on_command(self, message):
        """只接收已接受动作的有限输出；理想底盘仍遵守原0.8/1.0命令边界。"""
        if self.goal_handle is None:
            return
        values = [float(message.linear.x), float(message.angular.z)]
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError('MPPI返回非有限候选速度')
        self.raw, self.command_t = values, self.t

    def accepted(self, future, sequence):
        """记录当前滚动路径的接受状态；旧回调不参与后续速度有效性判断。"""
        handle = future.result()
        if sequence != self.sequence:
            return
        self.pending = None
        if not handle.accepted:
            self.rejections += 1
            return
        self.goal_handle = handle
        if self.first_accepted is None:
            self.first_accepted = self.t
        handle.get_result_async().add_done_callback(lambda result: self.finished(result, sequence))

    def finished(self, future, sequence):
        """保留动作结果；动作中止与命令时间过期是不同的诊断事实。"""
        response = future.result()
        payload = response.result
        self.results.append(dict(t=self.t, sequence=sequence, current=sequence == self.sequence,
                                 status=response.status,
                                 error_code=getattr(payload, 'error_code', None),
                                 error_msg=getattr(payload, 'error_msg', None)))
        if sequence == self.sequence:
            self.goal_handle = None

    def publish_state(self):
        """发布同步时钟、理想运动、TF与全自由地图；全自由地图仅用于本隔离基准。"""
        stamp = self.stamp()
        self.clock_pub.publish(Clock(clock=stamp))
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id = stamp, 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x, transform.transform.translation.y = self.x, self.y
        transform.transform.rotation.z = math.sin(self.yaw / 2)
        transform.transform.rotation.w = math.cos(self.yaw / 2)
        self.tf.sendTransform(transform)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.actual
        self.odom_pub.publish(odom)
        reference = TwistStamped()
        reference.header.stamp, reference.header.frame_id = stamp, 'base_footprint'
        reference.twist.linear.x = .8
        self.speed_pub.publish(reference)
        self.braking_pub.publish(Float64MultiArray(data=[float(self.t),.25,.45,1.]))
        if self.t - self.last_map >= .15:
            grid = OccupancyGrid()
            grid.header.stamp, grid.header.frame_id = stamp, 'odom'
            grid.info.resolution, grid.info.width, grid.info.height = .1, 120, 120
            grid.info.origin.position.x, grid.info.origin.position.y = self.x - 6, -6.
            grid.info.origin.orientation.w = 1.
            grid.data = [0] * 14400
            self.map_pub.publish(grid)
            self.last_map = self.t

    def check_lifecycle(self):
        """动作端点发现不等于节点已激活；等待生命周期ACTIVE后才开始诊断路径。"""
        if self.lifecycle_state == 3 or self.lifecycle_pending is not None:
            return
        if self.t - self.last_lifecycle_query < .2 or not self.lifecycle_client.service_is_ready():
            return
        self.last_lifecycle_query = self.t
        self.lifecycle_pending = self.lifecycle_client.call_async(GetState.Request())
        self.lifecycle_pending.add_done_callback(self.lifecycle_received)

    def lifecycle_received(self, future):
        """记录服务返回状态；用真实生命周期响应排除启动竞态。"""
        self.lifecycle_state = future.result().current_state.id
        self.lifecycle_pending = None

    def update_path(self):
        """每0.4秒送入沿固定世界直线的滚动参考，改变长度但不加入目标速度代价。"""
        if self.lifecycle_state != 3 or self.pending is not None or self.t - self.last_sent < .4 or not self.client.server_is_ready():
            return
        goal = FollowPath.Goal()
        goal.controller_id, goal.goal_checker_id = 'FollowPath', 'follow_goal'
        goal.path.header.stamp, goal.path.header.frame_id = self.stamp(), 'odom'
        count = math.ceil(self.reference_m / .08)
        for index in range(count + 1):
            pose = PoseStamped()
            pose.header = goal.path.header
            pose.pose.position.x = self.x + self.reference_m * index / count
            pose.pose.position.y = self.y if index == 0 else 0.
            pose.pose.orientation.w = 1.
            goal.path.poses.append(pose)
        self.sequence += 1
        self.last_sent = self.t
        sequence = self.sequence
        self.pending = self.client.send_goal_async(goal)
        self.pending.add_done_callback(lambda future: self.accepted(future, sequence))

    def tick(self, dt):
        """按实际墙钟推进理想积分，不通过加快仿真时钟改变控制器的有效运行频率。"""
        self.t += dt
        usable = self.goal_handle is not None and 0 <= self.t - self.command_t <= .3
        v, w = self.raw if usable else (0., 0.)
        self.actual = [max(0., min(.8, v)), max(-1., min(1., w))]
        self.x += self.actual[0] * math.cos(self.yaw) * dt
        self.y += self.actual[0] * math.sin(self.yaw) * dt
        self.yaw += self.actual[1] * dt
        self.publish_state()
        self.check_lifecycle()
        self.update_path()
        if self.first_accepted is not None and self.t - self.last_sample >= .05:
            self.rows.append(dict(t=self.t, elapsed=self.t - self.first_accepted,
                                  x=self.x, y=self.y, yaw=self.yaw,
                                  raw_command=self.raw.copy(), actual=self.actual.copy(),
                                  controller_active=self.goal_handle is not None, command_age=self.t - self.command_t,
                                  reference_length_m=self.reference_m))
            self.last_sample = self.t


def measure_case(name, profile, directory, prefix):
    """每场独立启动标准C++控制器，采样后核实退出，并在下一场覆盖日志前保存证据。"""
    original = mppi_runtime.parameters
    configuration = original()
    controller = configuration['/go2_follow_mppi/controller_server']['ros__parameters']
    controller['FollowPath'].update(profile['overrides'])
    mppi_runtime.parameters = lambda: configuration
    runtime, rig = mppi_runtime.MppiRuntime(), SpeedRig(profile['reference_m'], name)
    executor = SingleThreadedExecutor()
    executor.add_node(rig)
    summary = dict(case=name, profile=profile, parameters=configuration, samples=0,
                   source_sha256={file: hashlib.sha256((Path(__file__).resolve().parents[1] / file).read_bytes()).hexdigest()
                                  for file in ('mppi_runtime.py', 'navigation_config.py', 'tests/check_mppi_speed.py')})
    started = time.monotonic()
    try:
        runtime.start()
        summary['owned_nav2_processes'] = runtime.records.copy()
        previous = time.monotonic()
        while time.monotonic() - started < 45:
            executor.spin_once(timeout_sec=.01)
            now = time.monotonic()
            dt, previous = now - previous, now
            rig.tick(dt)
            if rig.first_accepted is not None and rig.t - rig.first_accepted >= 16:
                break
        else:
            raise TimeoutError('MPPI速度隔离测试未在预算内完成')
        intervals = [(a, b['t'] - a['t']) for a, b in zip(rig.rows, rig.rows[1:])
                     if 6 <= a['elapsed'] < 15]
        seconds = sum(dt for _, dt in intervals)
        if seconds < 8:
            raise RuntimeError('稳态速度采样不足')
        summary.update(observed_steady_seconds=seconds,
                       mean_raw_vx=sum(row['raw_command'][0] * dt for row, dt in intervals) / seconds,
                       mean_actual_vx=sum(row['actual'][0] * dt for row, dt in intervals) / seconds,
                       inactive_ratio=sum(dt for row, dt in intervals if not row['controller_active']) / seconds,
                       final_position=[rig.x, rig.y], final_yaw=rig.yaw, completed=True)
    except Exception as error:
        summary.update(completed=False, error=f'{type(error).__name__}: {error}')
    finally:
        runtime.close()
        # 保存每场日志，后续场景启动会覆盖生产入口同名的原生日志。
        for node in ('controller_server', 'lifecycle_manager'):
            log = mppi_runtime.ROOT / 'logs' / f'mppi-{node}.log'
            if log.exists():
                shutil.copy2(log, directory / f'{prefix}-{name}-{node}.log')
        summary.update(samples=len(rig.rows), action_results=rig.results, wall_elapsed_s=time.monotonic() - started,
                       lifecycle_state=rig.lifecycle_state, rejected_goals=rig.rejections,
                       sent_goals=rig.sequence, pending_goal=rig.pending is not None,
                       server_ready=rig.client.server_is_ready())
        directory.joinpath(f'{prefix}-{name}.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        directory.joinpath(f'{prefix}-{name}-samples.json').write_text(json.dumps(rig.rows, ensure_ascii=False), encoding='utf-8')
        executor.remove_node(rig)
        rig.client.destroy()
        rig.destroy_node()
        executor.shutdown()
        mppi_runtime.parameters = original
    print(json.dumps({key: summary.get(key) for key in ('case', 'completed', 'mean_raw_vx', 'mean_actual_vx', 'inactive_ratio', 'error')}, ensure_ascii=False), flush=True)
    return summary


def main():
    """独占本实验台的ROS域和物理资源，输出隔离基准；不将结果当作实际导航通过证明。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', default='speed-chain-mppi-20261006')
    parser.add_argument('--cases', choices=list(PROFILES), nargs='+', default=list(PROFILES))
    args = parser.parse_args()
    directory = mppi_runtime.ROOT / 'artifacts'
    directory.mkdir(exist_ok=True)
    with (mppi_runtime.ROOT / 'follow_demo/runtime.lock').open('a+') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('请先关闭跟随实验台；隔离诊断不能与人工验收同时运行。')
        report = dict(kind='isolated_mppi_speed_diagnostic', pid=os.getpid(),
                      started_at_utc=datetime.now(timezone.utc).isoformat(),
                      plant='理想积分底盘，不包含步态、相机、UWB与统一执行保护',
                      geometry='全自由地图，仅供隔离基准；没有改生产导航地图', cases=[])
        print(json.dumps(dict(pid=os.getpid(), cases=args.cases)), flush=True)
        try:
            for name in args.cases:
                # 每场重建ROS上下文，避免上场DDS动作端点和时钟回退干扰下场。
                rclpy.init()
                try:
                    report['cases'].append(measure_case(name, PROFILES[name], directory, args.prefix))
                finally:
                    rclpy.shutdown()
        finally:
            report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
            report['all_completed'] = len(report['cases']) == len(args.cases) and all(case['completed'] for case in report['cases'])
            directory.joinpath(f'{args.prefix}.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        return 0 if report['all_completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
