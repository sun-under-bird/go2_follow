"""本机跟随演示入口：物理仿真、ROS 控制节点与可视化面板。"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time
import traceback
from typing import Literal, Optional
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel, Field, model_validator
import rclpy
from rclpy.executors import SingleThreadedExecutor
import uvicorn
from .simulation import ROOT, Simulation
from .ros_nodes import SensorBridge, FollowerNode, Telemetry, TfReceiver
from .mppi_runtime import MppiRuntime
from .executor_pulse import ExecutorPulse
from .camera_profile import DEFAULT_CAMERA_PITCH_DEG


class Command(BaseModel):
    """限定页面可执行的仿真操作及目标活动范围。"""
    action: Literal['resume', 'pause', 'estop', 'reset', 'signal_on', 'signal_off', 'hold', 'straight', 'circle', 'waypoint', 'speed',
                    'scenario', 'route', 'confirm_clearance', 'depth_on', 'depth_off']
    scenario: Optional[Literal['open', 'long_wall', 'consecutive', 'corner', 'blocked',
                               'square_loop', 'slalom_loop', 'wall_loop']] = None
    x: Optional[float] = Field(default=None, ge=-18, le=18, allow_inf_nan=False)
    y: Optional[float] = Field(default=None, ge=-18, le=18, allow_inf_nan=False)
    speed: Optional[float] = Field(default=None, ge=0.05, le=0.80, allow_inf_nan=False)

    @model_validator(mode='after')
    def require_waypoint(self):
        """地图目标必须包含完整坐标，避免不完整输入进入物理线程。"""
        if self.action == 'waypoint' and (self.x is None or self.y is None):
            raise ValueError('地图目标需要 x、y 坐标')
        if self.action == 'scenario' and self.scenario is None:
            raise ValueError('场景切换需要选择场景')
        return self


class SchedulingMonitor:
    """仅在 ROS 回调空档时记录线程栈，诊断不参与控制或输入失效判定。"""
    def __init__(self, bridge, follower, simulation, path, gap_seconds=.6,
                 event_limit=32, byte_limit=2 * 1024 * 1024):
        """准备独立日志线程和双重容量上限，不占用 ROS 或遥测文件锁。"""
        self.bridge, self.follower, self.simulation = bridge, follower, simulation
        self.path = Path(path)
        self.gap_seconds = gap_seconds
        self.event_limit, self.byte_limit = event_limit, byte_limit
        self.quit = threading.Event()
        self.events = self.written_bytes = self.dropped = 0
        self.clock_state = None
        self.clock_jump_count = 0
        self.clock_jumps = []
        self.error = ''
        self.thread = threading.Thread(target=self.run, name='go2-schedule-monitor', daemon=True)

    def start(self):
        """在 ROS 与 Nav2 启动完成后启用，尚未进入过两类回调时不判空档。"""
        self.thread.start()

    def callback_states(self):
        """读取原子发布的小字典，避免监视器等待可能正被阻塞的业务锁。"""
        return dict(bridge=self.bridge.callback_trace.read(), control=self.follower.callback_trace.read())

    def stale_callbacks(self, states, now):
        """只用于触发诊断；0.6 秒不替代现有的任何健康或停车门槛。"""
        if any(state['entry_wall'] is None for state in states.values()):
            return []
        return [name for name, state in states.items() if now - state['entry_wall'] > self.gap_seconds]

    def sample_clock(self):
        """监视日期与单调时间的差值；只记诊断，不把校时跳变用于控制。"""
        before = time.monotonic()
        system_wall = time.time()
        after = time.monotonic()
        monotonic_wall = (before + after) / 2
        offset = system_wall - monotonic_wall
        previous = self.clock_state
        delta = 0.0 if previous is None else offset - previous['system_minus_monotonic_s']
        self.clock_state = dict(system_wall=system_wall, monotonic_wall=monotonic_wall,
                                system_minus_monotonic_s=offset, offset_delta_s=delta,
                                minimum_offset_delta_s=min(delta, 0.0 if previous is None else previous['minimum_offset_delta_s']),
                                maximum_offset_delta_s=max(delta, 0.0 if previous is None else previous['maximum_offset_delta_s']),
                                capture_span_s=after - before)
        if previous is not None and abs(delta) > .05:
            # 只保留最近八次跳变，避免正常采样变成高频日志或无界历史。
            self.clock_jump_count += 1
            self.clock_jumps = (self.clock_jumps + [dict(self.clock_state)])[-8:]
        return dict(self.clock_state)

    def capture(self, now, states, stale):
        """收集全部 Python 线程名和栈，以及独立物理快照时间，禁止获取模拟器锁。"""
        frames = sys._current_frames()
        names = {thread.ident: thread for thread in threading.enumerate()}
        stacks = []
        # 物理线程以整份字典替换 snapshot；读引用即可，不能为诊断等待其业务锁。
        snapshot = self.simulation.snapshot
        for ident, frame in frames.items():
            thread = names.get(ident)
            stack = traceback.extract_stack(frame, limit=48)
            stacks.append(dict(ident=ident, native_id=None if thread is None else thread.native_id,
                               name='unknown' if thread is None else thread.name,
                               stack=[dict(file=item.filename[-500:], line=item.lineno,
                                           function=item.name[:200]) for item in stack]))
        return dict(wall=now, pid=os.getpid(), event=self.events + 1, stale=stale,
                    clock=dict(self.clock_state) if self.clock_state is not None else self.sample_clock(),
                    clock_jump_count=self.clock_jump_count, clock_jumps=list(self.clock_jumps),
                    callbacks=states,
                    physical={key: snapshot.get(key) for key in ('t', 'epoch', 'x', 'y', 'watchdog')},
                    threads=stacks, captured_thread_count=len(frames))

    def run(self):
        """每次空档只写一次，恢复后重新允许；事件限频且日志大小有界。"""
        latched, last_event = False, -float('inf')
        try:
            with self.path.open('wb') as log:
                while not self.quit.wait(.05):
                    self.sample_clock()
                    now, states = time.monotonic(), self.callback_states()
                    stale = self.stale_callbacks(states, now)
                    if not stale:
                        latched = False
                        continue
                    if latched or now - last_event < 1.0:
                        continue
                    latched, last_event = True, now
                    if self.events >= self.event_limit:
                        self.dropped += 1
                        continue
                    event = self.capture(now, states, stale)
                    content = (json.dumps(event, ensure_ascii=False) + '\n').encode('utf-8')
                    if self.written_bytes + len(content) > self.byte_limit:
                        self.dropped += 1
                        continue
                    log.write(content)
                    log.flush()
                    self.events += 1
                    self.written_bytes += len(content)
        except Exception as error:
            # 诊断失败单独可见，但不能反向改变速度、安全门槛或 ROS 调度器。
            self.error = str(error)

    def diagnostics(self):
        """返回日志位置、容量与线程状态，让验收能区分无事件和记录器故障。"""
        return dict(path=str(self.path), gap_seconds=self.gap_seconds, events=self.events,
                    written_bytes=self.written_bytes, dropped=self.dropped, error=self.error,
                    clock=None if self.clock_state is None else dict(self.clock_state),
                    clock_jump_count=self.clock_jump_count, clock_jumps=list(self.clock_jumps),
                    worker_alive=self.thread.is_alive())

    def close(self):
        """显式停并检查唯一监视线程；不在其他线程关闭其日志文件。"""
        self.quit.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=5)
        if self.thread.is_alive():
            raise RuntimeError('调度诊断监视线程未退出，独立日志文件尚未确认关闭')


class Runtime:
    """管理本次演示拥有的线程与 ROS 节点，退出时统一释放。"""
    def __init__(self, render=True, execution_mode='rate', planning_mode='trail', observation_mode='camera',
                 executor_wake_mode='steady', search_execution_mode='process',
                 camera_pitch_deg=DEFAULT_CAMERA_PITCH_DEG):
        """创建模拟器和两个 ROS 节点，控制器没有对模拟器的直接引用。"""
        self.telemetry = Telemetry()
        self.planning_mode = planning_mode
        self.observation_mode = observation_mode
        self.executor_wake_mode = executor_wake_mode
        self.search_execution_mode = search_execution_mode
        self.shutdown_lock = threading.Lock()
        self.shutdown_requested = False
        self.simulation = Simulation(render=render, execution_mode=execution_mode, camera_pitch_deg=camera_pitch_deg)
        rclpy.init()
        self.bridge = SensorBridge(self.simulation, self.telemetry)
        self.tf_receiver = TfReceiver()
        self.follower = FollowerNode(use_follow_intent=planning_mode == 'trail',
                                     use_camera_observation=observation_mode == 'camera', tf_buffer=self.tf_receiver.buffer,
                                     use_process_search=search_execution_mode == 'process')
        self.mppi = MppiRuntime()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.follower)
        # 传感器桥独立调度，导航处理深度时仍能发布新里程计和接收最终速度。
        self.sensor_executor = SingleThreadedExecutor()
        self.sensor_executor.add_node(self.bridge)
        # TF 接收只写内部缓冲；控制器依然由唯一执行器线程持有，不并发修改 core。
        self.tf_executor = SingleThreadedExecutor()
        self.tf_executor.add_node(self.tf_receiver)
        self.executor_pulse = ExecutorPulse((self.executor, self.sensor_executor, self.tf_executor))
        self.ros_thread = threading.Thread(target=self.executor.spin, name='go2-ros', daemon=True)
        self.sensor_thread = threading.Thread(target=self.sensor_executor.spin, name='go2-sensors', daemon=True)
        self.tf_thread = threading.Thread(target=self.tf_executor.spin, name='go2-tf', daemon=True)
        self.ros_thread.start()
        self.sensor_thread.start()
        self.tf_thread.start()
        if executor_wake_mode == 'steady':
            self.executor_pulse.start()
        self.simulation.start()
        self.scheduling_monitor = SchedulingMonitor(
            self.bridge, self.follower, self.simulation,
            self.telemetry.path.with_name(self.telemetry.path.stem + '-scheduling.jsonl'))
        try:
            self.mppi.start()
            self.scheduling_monitor.start()
        except Exception:
            self.close()
            raise

    def request_shutdown(self):
        """重复关闭请求只发一次退出信号，避免第二次中断正在回收的渲染和 ROS 线程。"""
        with self.shutdown_lock:
            if self.shutdown_requested:
                return False
            self.shutdown_requested = True
            self.simulation.action('estop')
            threading.Timer(.3, lambda: os.kill(os.getpid(), signal.SIGINT)).start()
            return True

    def close(self):
        """先停物理步进，再停通信，避免留下仍执行旧命令的模拟器。"""
        simulation_error = None
        try:
            self.executor_pulse.close()
        except Exception as error:
            simulation_error = error
        try:
            self.scheduling_monitor.close()
        except Exception as error:
            # 即使诊断线程释放失败，也必须继续清理物理及专属 ROS/Nav2 资源。
            simulation_error = error
        try:
            self.simulation.close()
        except Exception as error:
            # 渲染线程超时也要继续清专属 ROS/Nav2 资源，再让进程明确失败退出。
            if simulation_error is None:
                simulation_error = error
        self.executor.shutdown(timeout_sec=5)
        self.sensor_executor.shutdown(timeout_sec=5)
        self.tf_executor.shutdown(timeout_sec=5)
        self.ros_thread.join(timeout=5)
        self.sensor_thread.join(timeout=5)
        self.tf_thread.join(timeout=5)
        # ROS 不再接入结果后，等待唯一深度工作线程退出，再释放导航器。
        self.follower.close_depth_worker()
        self.follower.core.close()
        self.mppi.close()
        self.bridge.destroy_node()
        self.follower.destroy_node()
        self.tf_receiver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.telemetry.close()
        if simulation_error is not None:
            raise simulation_error


def make_app(runtime):
    """创建仅绑定本机的 HTTP 页面、图像、操作和日志接口。"""
    app = FastAPI(title='Go2 平地有限视野跟随演示')

    @app.get('/')
    def index():
        """返回无外部 CDN 依赖的本地可视化页面。"""
        return FileResponse(Path(__file__).parent / 'static/index.html')

    @app.get('/api/state')
    def state():
        """返回当前物理状态、控制诊断及曲线数据。"""
        payload = runtime.telemetry.read()
        # 模式启动后固定，不能在运行中切换而保留另一模式的航向与历史状态。
        with runtime.simulation.lock:
            execution = dict(runtime.simulation.snapshot.get('execution', {}))
        payload.update(app_id='go2_follow_demo', ros_alive=runtime.ros_thread.is_alive() and runtime.sensor_thread.is_alive() and runtime.tf_thread.is_alive(), physics_alive=not runtime.simulation.quit.is_set(),
                       scheduling_monitor=runtime.scheduling_monitor.diagnostics(),
                       executor_wake_mode=runtime.executor_wake_mode,
                       executor_pulse=runtime.executor_pulse.diagnostics(),
                       search_execution_mode=runtime.search_execution_mode,
                       execution_mode=runtime.simulation.policy.yaw_mode, execution=execution,
                       planning_mode=runtime.planning_mode,
                       observation_mode=runtime.observation_mode,
                       camera_pitch_deg=runtime.simulation.camera_pitch_deg,
                       mppi_alive=all(p.poll() is None for p in runtime.mppi.processes),
                       error=runtime.simulation.error or runtime.simulation.render_error or payload.get('error') or
                       runtime.executor_pulse.diagnostics().get('error') or
                       (None if all(p.poll() is None for p in runtime.mppi.processes) else 'Nav2 MPPI 子进程退出，请查看 mppi 日志'))
        return payload

    @app.get('/api/frame/{name}')
    def frame(name: Literal['left', 'right', 'scene']):
        """读取已渲染的 JPEG，不在请求线程创建额外 OpenGL 上下文。"""
        with runtime.simulation.lock:
            data = runtime.simulation.images.get(name)
        if data is None:
            return Response(status_code=204)
        return Response(data, media_type='image/jpeg', headers={'Cache-Control': 'no-store'})

    @app.post('/api/command')
    def command(request: Command):
        """把经过边界验证的人工操作提交给仿真场景。"""
        if runtime.simulation.error:
            raise HTTPException(409, '物理线程异常，请查看日志并重启演示')
        runtime.simulation.action(request.action, request.x, request.y, request.speed, request.scenario)
        return {'ok': True, 'action': request.action}

    @app.get('/api/log.csv')
    def download_log():
        """导出记录的固定快照，避免写入中的文件改变 HTTP 响应长度。"""
        content = runtime.telemetry.download_log()
        return Response(content, media_type='text/csv',
                        headers={'Content-Disposition': f'attachment; filename="{runtime.telemetry.path.name}"'})

    @app.post('/api/shutdown')
    def shutdown():
        """先停止跟随，再结束本次演示服务。"""
        runtime.request_shutdown()
        return {'ok': True}

    return app


def main():
    """启动独占的本机仿真实例；同一用户不可同时启动两个演示控制器。"""
    parser = argparse.ArgumentParser(description='Go2 平地有限视野跟随演示')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--no-render', action='store_true', help='仅供无图形的自动化检查使用')
    parser.add_argument('--execution-mode', choices=('heading', 'velocity', 'rate'), default='rate',
                        help='独立执行模式对照；只改变角速度输入生成，不改规划和安全门槛')
    parser.add_argument('--planning-mode', choices=('annulus', 'trail'), default='trail',
                        help='独立终点语义对照：距离环或按目标历史路线落后的跟随点；均须搜索真实可通行地图')
    parser.add_argument('--observation-mode', choices=('cone', 'camera'), default='camera',
                        help='观察几何对照：旧二维扇区或真实 CameraInfo/TF 对下一段包络的地面支持')
    parser.add_argument('--executor-wake-mode', choices=('native', 'steady'), default='steady',
                        help='ROS 等待对照：底层原生等待或独立单调 guard condition 唤醒；不改变任何失效门槛')
    parser.add_argument('--search-execution-mode', choices=('thread', 'process'), default='process',
                        help='规划隔离对照：共享 Python 线程或独立 spawn 进程；地图及安全门槛一致')
    parser.add_argument('--camera-pitch-deg', type=float, default=DEFAULT_CAMERA_PITCH_DEG,
                        help='显式仿真安装俯角，向下为正、范围0～40度；不代表实机标定')
    arguments = parser.parse_args()
    lock = (ROOT / 'follow_demo/runtime.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('跟随演示已经运行，请打开 http://localhost:8765')
    runtime = Runtime(render=not arguments.no_render, execution_mode=arguments.execution_mode,
                      planning_mode=arguments.planning_mode, observation_mode=arguments.observation_mode,
                      executor_wake_mode=arguments.executor_wake_mode,
                      search_execution_mode=arguments.search_execution_mode,
                      camera_pitch_deg=arguments.camera_pitch_deg)
    try:
        print(f'Go2 跟随演示：http://localhost:{arguments.port}，执行模式：{arguments.execution_mode}，日志：{runtime.telemetry.path}', flush=True)
        uvicorn.run(make_app(runtime), host='127.0.0.1', port=arguments.port, log_level='warning', access_log=False)
    finally:
        runtime.close()
        lock.close()


if __name__ == '__main__':
    main()
