"""MuJoCo 真实物理步进、虚拟目标和前视双目渲染；不直接驱动机器人位姿。"""
from collections import deque
from pathlib import Path
import math
import multiprocessing
import os
import threading
import time
import traceback
import xml.etree.ElementTree as ET
import cv2
import mujoco
import numpy as np
from .ui_render import render_worker
from .policy import Policy, yaw_from_quaternion
from .camera_profile import (WIDTH, HEIGHT, FOVY, BASELINE, CAMERA_CENTER, camera_position,
                             mounting_geometry, DEFAULT_CAMERA_PITCH_DEG)
from .scenarios import SCENARIOS
from .scenario_route import RouteWalker
from .navigation_config import MAX_NAVIGATION_SPEED,MAX_NAVIGATION_TURN

ROOT = Path.home() / 'go2_sim'


def make_scene(destination, camera_pitch_deg=DEFAULT_CAMERA_PITCH_DEG):
    """从锁定场景生成独立副本，加入具名双目相机、原始 IMU 和可移动目标标记。"""
    source = ROOT / 'third_party/unitree_mujoco/unitree_robots/go2'
    destination.mkdir(parents=True, exist_ok=True)
    robot = ET.parse(source / 'go2.xml')
    robot.find('compiler').set('meshdir', str(source / 'assets'))
    base = robot.find(".//body[@name='base_link']")
    axes, _, imu_rotation = mounting_geometry(camera_pitch_deg)
    for side in ('left', 'right'):
        ET.SubElement(base, 'camera', name=f'front_{side}',
                      pos=' '.join(str(value) for value in camera_position(side)),
                      xyaxes=' '.join(str(value) for value in axes), fovy=str(FOVY))
    # 尚无实测 IMU 外参，先放在双目中点；不把这个位置当作 D435i 厂家外参。
    # IMU 与相机安装一同旋转；MuJoCo 的四元数顺序为 wxyz。
    ET.SubElement(base, 'site', name='camera_imu', pos=' '.join(str(value) for value in CAMERA_CENTER),
                  quat=' '.join(str(value) for value in (imu_rotation[3], *imu_rotation[:3])),
                  size='0.005', rgba='0 0 0 0')
    ET.SubElement(robot.find('sensor'), 'gyro', name='camera_gyro', site='camera_imu')
    ET.SubElement(robot.find('sensor'), 'accelerometer', name='camera_acc', site='camera_imu')
    robot.write(destination / 'go2_camera.xml', encoding='utf-8')
    scene = ET.parse(source / 'scene.xml')
    scene.find('include').set('file', str(destination / 'go2_camera.xml'))
    scene.find('visual/global').set('offwidth', '960')
    scene.find('visual/global').set('offheight', '540')
    # 深度缓冲多重采样会取最近子像素，导致深度和像素中心射线不一致。
    # 关闭离屏MSAA，保证每个深度测量与发布的CameraInfo代表同一条射线。
    quality = scene.find('visual/quality')
    if quality is None:
        quality = ET.SubElement(scene.find('visual'), 'quality')
    quality.set('offsamples', '0')
    for scenario, specification in SCENARIOS.items():
        for index, (x, y, sx, sy, height) in enumerate(specification['boxes']):
            obstacle = ET.SubElement(scene.find('worldbody'), 'body', name=f'obstacle_{scenario}_{index}',
                                     mocap='true', pos=f'{x} {y} -20')
            ET.SubElement(obstacle, 'geom', name=f'obstacle_geom_{scenario}_{index}', type='box',
                          size=f'{sx} {sy} {height / 2}', rgba='0.35 0.49 0.65 1', contype='1', conaffinity='1')
    person = ET.SubElement(scene.find('worldbody'), 'body', name='follow_target', mocap='true', pos='2.6 0 0')
    # 目标是用于跟随的可视化标记，首版不把它作为可推挤的人体动力学模型。
    geoms = [('capsule', '0 0 0.8 0 0 1.25', '0.17', '1 0.46 0.12 1'),
             ('capsule', '0 0.10 0.12 0 0.10 0.70', '0.075', '0.1 0.17 0.27 1'),
             ('capsule', '0 -0.10 0.12 0 -0.10 0.70', '0.075', '0.1 0.17 0.27 1')]
    for kind, ends, size, color in geoms:
        ET.SubElement(person, 'geom', type=kind, fromto=ends, size=size, rgba=color, contype='0', conaffinity='0', group='4')
    ET.SubElement(person, 'geom', type='sphere', pos='0 0 1.53', size='0.13', rgba='1 0.78 0.6 1', contype='0', conaffinity='0', group='4')
    scene.write(destination / 'scene.xml', encoding='utf-8')
    return destination / 'scene.xml'


class Simulation:
    """唯一持有可写 mjData 的物理线程，渲染使用独立数据快照。"""
    def __init__(self, render=True, execution_mode='heading', camera_pitch_deg=DEFAULT_CAMERA_PITCH_DEG):
        """加载场景与显式指定的执行模式，准备线程、快照与控制看门狗。"""
        self.camera_pitch_deg = float(camera_pitch_deg)
        self.model = mujoco.MjModel.from_xml_path(str(make_scene(ROOT / 'follow_demo/runtime', self.camera_pitch_deg)))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.policy = Policy(self.model, self.data, ROOT / 'third_party/capo-sim-assets/sim_patch/models/go2_policy.onnx',
                             yaw_mode=execution_mode)
        self.lock = threading.RLock()
        self.quit = threading.Event()
        self.request_reset = False
        self.epoch = 0
        self.epoch_origin = 0.0
        self.target = np.array([2.6, 0.0])
        self.target_velocity = np.zeros(2)
        self.target_mode = 'hold'
        self.target_goal = self.target.copy()
        self.target_speed = 0.80
        self.mode_angle = 0.0
        self.mode_origin = self.target.copy()
        self.command = np.zeros(3)
        self.command_received = 0.0
        self.signal_valid = True
        self.follow_enabled = False
        self.emergency = False
        self.snapshot = {}
        self.images = {}
        self.frames = None
        self.depth_frame = None
        self.render_diagnostics = dict(epoch=0, **{name: dict(target_hz=rate, delivered=0, skipped=0,
                                                           last_render_ms=0.0, maximum_render_ms=0.0,
                                                           actual_hz=0.0, last_capture_t=None, last_delivery_wall=None)
                                                  for name, rate in [('depth', 8.0), ('stereo', 2.0), ('overview', 1.0)]})
        self.render_delivery_times = {name: deque(maxlen=32) for name in ('depth', 'stereo', 'overview')}
        self.depth_enabled = True
        self.scenario = 'open'
        self.next_scenario = 'open'
        self.route_walker = None
        self.clearance_confirmed = False
        self.obstacle_contacts = 0
        self.render_error = None
        self.error = None
        self.render_enabled = render
        self.trace = deque(maxlen=3000)
        self.start_wall = time.monotonic()
        self.threads = []
        self.ui_process = None
        self.base_id = self.model.body('base_link').id
        self.foot_ids = [self.model.body(name + '_foot').id for name in ('FR', 'FL', 'RR', 'RL')]
        self.target_id = int(self.model.body_mocapid[self.model.body('follow_target').id])
        self.obstacle_ids = {}
        self.obstacle_geoms = set()
        for scenario, specification in SCENARIOS.items():
            self.obstacle_ids[scenario] = []
            for index, _ in enumerate(specification['boxes']):
                self.obstacle_ids[scenario].append(int(self.model.body_mocapid[self.model.body(f'obstacle_{scenario}_{index}').id]))
                self.obstacle_geoms.add(self.model.geom(f'obstacle_geom_{scenario}_{index}').id)

    def start(self):
        """启动物理、专用深度和低频界面线程；各渲染线程拥有私有数据与上下文。"""
        self.threads = [threading.Thread(target=self.run, name='go2-physics', daemon=True)]
        if self.render_enabled:
            self.threads.append(threading.Thread(target=self.render, name='go2-depth-render', daemon=True))
            self.threads.append(threading.Thread(target=self.render_ui, name='go2-ui-render', daemon=True))
        for thread in self.threads:
            thread.start()

    def close(self):
        """通知并核实本实例线程退出；OpenGL 由所属线程释放，超时明确报错。"""
        self.quit.set()
        for thread in self.threads:
            thread.join(timeout=8)
        alive = [thread.name for thread in self.threads if thread.is_alive()]
        if alive:
            raise RuntimeError('仿真线程退出超时，不能确认其资源已释放：' + ', '.join(alive))

    def set_command(self, vx, vy, wz):
        """接收 ROS 速度命令；越界和非有限输入均在进入策略前处理。"""
        with self.lock:
            self.command = (np.clip([vx,vy,wz],[-.5,-.5,-MAX_NAVIGATION_TURN],
                                   [MAX_NAVIGATION_SPEED,.5,MAX_NAVIGATION_TURN])
                            if np.isfinite([vx,vy,wz]).all() else np.zeros(3))
            self.command_received = time.monotonic()

    def action(self, action, x=None, y=None, speed=None, scenario=None):
        """处理经过 HTTP 模型校验的目标移动、信号中断、暂停、急停和场景重置。"""
        with self.lock:
            if speed is not None:
                self.target_speed = speed
            if action in ('straight', 'circle', 'hold', 'waypoint', 'route'):
                previous_mode = self.target_mode
                self.target_mode = action
                self.mode_angle = 0.0
                self.mode_origin = self.target.copy()
                if action == 'route':
                    # 重复点击或目标停下后恢复沿路线，都保留当前进度；换运动模式后重新接入起点。
                    if self.route_walker is None or previous_mode not in ('route', 'hold'):
                        self.route_walker = RouteWalker(SCENARIOS[self.scenario], self.target)
                elif action != 'hold':
                    self.route_walker = None
                if action == 'waypoint':
                    self.target_goal = np.array([x, y])
            elif action == 'signal_off':
                self.signal_valid = False
            elif action == 'signal_on':
                self.signal_valid = True
            elif action == 'resume':
                self.follow_enabled, self.emergency = True, False
            elif action == 'pause':
                self.follow_enabled = False
                self.command[:] = 0
            elif action == 'estop':
                self.emergency, self.follow_enabled = True, False
                self.target_mode = 'hold'
                self.command[:] = 0
            elif action == 'reset':
                self.request_reset = True
            elif action == 'scenario':
                self.next_scenario, self.request_reset = scenario, True
            elif action == 'confirm_clearance' and not self.follow_enabled:
                self.clearance_confirmed = True
            elif action == 'depth_off':
                self.depth_enabled = False
            elif action == 'depth_on':
                self.depth_enabled = True

    def reset(self):
        """重置场景，同时保持 ROS 仿真时间单调，使用 epoch 清除估计器旧状态。"""
        self.epoch_origin += self.data.time
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.policy.reset(self.data)
        self.epoch += 1
        self.target[:] = SCENARIOS[self.next_scenario].get('start', [2.0, 0])
        self.target_velocity[:] = 0
        self.target_mode = 'hold'
        self.route_walker = None
        self.follow_enabled, self.emergency, self.signal_valid = False, False, True
        self.command[:] = 0
        self.command_received = 0
        self.request_reset = False
        self.scenario = self.next_scenario
        self.depth_enabled, self.clearance_confirmed = True, False
        self.depth_frame, self.frames = None, None
        self.obstacle_contacts = 0
        for name, indices in self.obstacle_ids.items():
            for index, body in enumerate(indices):
                x, y, sx, sy, height = SCENARIOS[name]['boxes'][index]
                self.data.mocap_pos[body] = [x, y, height / 2 if name == self.scenario else -20]
        self.trace.clear()

    def move_target(self, dt):
        """按受限速度移动虚拟目标；地图点击只设目的地，不瞬移目标位置。"""
        old = self.target.copy()
        if self.target_mode == 'straight':
            self.target[0] += self.target_speed * dt
        elif self.target_mode == 'circle':
            # 积分路程而不是用新速度乘累计时间，调整速度时保持目标位置连续。
            self.mode_angle += self.target_speed * dt / 3.0
            angle = self.mode_angle
            self.target[:] = self.mode_origin + [3 * math.sin(angle), 3 * (1 - math.cos(angle))]
        elif self.target_mode == 'route':
            self.target[:] = self.route_walker.advance(self.target_speed, dt)
            if self.route_walker.status()['finished']:
                self.target_mode = 'hold'
        elif self.target_mode == 'waypoint':
            difference = self.target_goal - self.target
            length = np.linalg.norm(difference)
            if length > 0.01:
                self.target += difference / length * min(length, self.target_speed * dt)
            else:
                self.target_mode = 'hold'
        self.target[:] = np.clip(self.target, -18, 18)
        self.target_velocity = (self.target - old) / dt
        self.data.mocap_pos[self.target_id] = [*self.target, 0]
        if np.linalg.norm(self.target_velocity) > 0.001:
            angle = math.atan2(self.target_velocity[1], self.target_velocity[0])
            self.data.mocap_quat[self.target_id] = [math.cos(angle / 2), 0, 0, math.sin(angle / 2)]

    def run(self):
        """按墙钟节奏执行真实物理步进，策略只通过电机力矩影响机身运动。"""
        dt = self.model.opt.timestep
        deadline = time.monotonic()
        try:
            while not self.quit.is_set():
                with self.lock:
                    if self.request_reset:
                        self.reset()
                        deadline = time.monotonic()
                    self.move_target(dt)
                    watchdog = time.monotonic() - self.command_received > 0.35
                    command = self.command.copy()
                    if watchdog or self.emergency or not self.follow_enabled:
                        command[:] = 0
                    self.policy.update(self.data, command)
                    self.policy.torque(self.data)
                    mujoco.mj_step(self.model, self.data)
                    # 每个物理步统计障碍接触，不能只在较慢的显示快照上抽样。
                    for contact in self.data.contact[:self.data.ncon]:
                        if contact.geom1 in self.obstacle_geoms or contact.geom2 in self.obstacle_geoms:
                            other = contact.geom2 if contact.geom1 in self.obstacle_geoms else contact.geom1
                            if int(self.model.geom_bodyid[other]) > 0 and other not in self.obstacle_geoms:
                                self.obstacle_contacts += 1
                                break
                    if not np.isfinite(self.data.qpos).all():
                        raise RuntimeError('物理状态出现非有限数值')
                    if int(round(self.data.time / dt)) % 5 == 0:
                        self.update_snapshot(command, watchdog)
                deadline += dt
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    self.quit.wait(remaining)
                elif remaining < -0.15:
                    # 图形或调度超时时降低实时倍率，不把积压的控制动作瞬间补发。
                    deadline = time.monotonic()
        except Exception:
            self.error = traceback.format_exc()
            self.quit.set()

    def update_snapshot(self, command, watchdog):
        """生成同一物理时刻的传感器、位置和渲染快照。"""
        quaternion = self.data.sensor('imu_quat').data.copy()
        yaw = yaw_from_quaternion(quaternion)
        position = self.data.xpos[self.base_id].copy()
        velocity = self.data.sensor('frame_vel').data.copy()
        rotation = self.data.xmat[self.base_id].reshape(3, 3)
        body_velocity = rotation.T @ velocity
        ready = self.data.time >= 3.0 and float(rotation[2, 2]) > 0.55 and position[2] > 0.15
        foot_force = np.zeros(4)
        contact_force = np.zeros(6)
        obstacle_contact = False
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            # 真实碰撞只进入验收数据，不作为导航器的提前避障信息。
            if contact.geom1 in self.obstacle_geoms or contact.geom2 in self.obstacle_geoms:
                other = contact.geom2 if contact.geom1 in self.obstacle_geoms else contact.geom1
                if int(self.model.geom_bodyid[other]) > 0 and other not in self.obstacle_geoms:
                    obstacle_contact = True
            bodies = (int(self.model.geom_bodyid[contact.geom1]), int(self.model.geom_bodyid[contact.geom2]))
            mujoco.mj_contactForce(self.model, self.data, index, contact_force)
            for foot, body in enumerate(self.foot_ids):
                if body in bodies:
                    foot_force[foot] += max(0.0, contact_force[0])
        execution = self.policy.execution_diagnostics()
        # HTTP 的 ROS 遥测缓存与物理快照可能不同步，诊断保留自身快照时间。
        execution['t'] = self.epoch_origin + float(self.data.time)
        self.snapshot = dict(t=self.epoch_origin + float(self.data.time), episode_t=float(self.data.time), epoch=self.epoch,
                             x=float(position[0]), y=float(position[1]), z=float(position[2]), yaw=yaw,
                             quaternion=quaternion, vx=float(body_velocity[0]), vy=float(body_velocity[1]),
                             wz=float(self.data.sensor('imu_gyro').data[2]), target=self.target.copy(),
                             target_velocity=self.target_velocity.copy(), target_mode=self.target_mode,
                             target_route=self.route_walker.status() if self.route_walker else None,
                             signal=self.signal_valid, enabled=self.follow_enabled, emergency=self.emergency,
                             clearance_confirmed=self.clearance_confirmed, depth_enabled=self.depth_enabled,
                             scenario=self.scenario, obstacle_contacts=self.obstacle_contacts,
                             ready=bool(ready), watchdog=bool(watchdog), applied=command.tolist(),
                             execution_mode=self.policy.yaw_mode, execution=execution,
                             q=self.policy.positions(self.data), dq=self.policy.velocities(self.data),
                             foot_force=foot_force,
                             gyro=self.data.sensor('imu_gyro').data.copy(), accel=self.data.sensor('imu_acc').data.copy(),
                             camera_gyro=self.data.sensor('camera_gyro').data.copy(), camera_acc=self.data.sensor('camera_acc').data.copy(),
                             qpos=self.data.qpos.copy(), qvel=self.data.qvel.copy(),
                             mocap_pos=self.data.mocap_pos.copy(), mocap_quat=self.data.mocap_quat.copy(),
                             rtf=(self.epoch_origin + self.data.time) / max(time.monotonic() - self.start_wall, 0.01))

    def render_state(self, drawing):
        """取一次冻结的物理快照写入调用线程的私有 mjData，不向模拟器写回。"""
        with self.lock:
            state = self.snapshot.copy()
        if not state:
            return None
        drawing.qpos[:] = state['qpos']
        drawing.qvel[:] = state['qvel']
        drawing.mocap_pos[:] = state['mocap_pos']
        drawing.mocap_quat[:] = state['mocap_quat']
        mujoco.mj_forward(self.model, drawing)
        return state

    def record_render_delivery(self, name, state, started):
        """已持有短缓存锁时更新采集时间、耗时与实际频率；指标不参与控制。"""
        if self.render_diagnostics['epoch'] != state['epoch']:
            for stream in self.render_delivery_times.values():
                stream.clear()
            self.render_diagnostics = dict(epoch=state['epoch'], **{
                stream: dict(target_hz=rate, delivered=0, skipped=0, last_render_ms=0.0,
                             maximum_render_ms=0.0, actual_hz=0.0, last_capture_t=None, last_delivery_wall=None)
                for stream, rate in [('depth', 8.0), ('stereo', 2.0), ('overview', 1.0)]})
        delivered_wall = time.monotonic()
        times = self.render_delivery_times[name]
        times.append(delivered_wall)
        diagnostic = self.render_diagnostics[name]
        duration_ms = (delivered_wall - started) * 1000
        diagnostic.update(delivered=diagnostic['delivered'] + 1, last_render_ms=duration_ms,
                          maximum_render_ms=max(diagnostic['maximum_render_ms'], duration_ms),
                          actual_hz=(len(times)-1) / max(times[-1]-times[0], 1e-9) if len(times)>1 else 0.0,
                          last_capture_t=state['t'], last_delivery_wall=delivered_wall)

    def render(self):
        """独立深度线程以 8 Hz 为目标，不等待双目、总览或 JPEG 的计算与文件 I/O。"""
        try:
            drawing = mujoco.MjData(self.model)
            options = mujoco.MjvOption()
            # UWB 标记仍无人体动力学，本阶段不将它渲染为导航障碍。
            options.geomgroup[4] = 0
            with mujoco.Renderer(self.model, height=HEIGHT // 2, width=WIDTH // 2) as camera:
                camera.enable_depth_rendering()
                while not self.quit.is_set():
                    started = time.monotonic()
                    state = self.render_state(drawing)
                    if state is not None and state['depth_enabled']:
                        camera.update_scene(drawing, camera='front_left', scene_option=options)
                        depth = camera.render().copy()
                        depth[(depth < .25) | (depth > 5.0) | ~np.isfinite(depth)] = np.nan
                        with self.lock:
                            # 时间始终取采集快照；重置与中断后不能交付先前在途深度。
                            if state['epoch'] == self.epoch and self.depth_enabled:
                                self.depth_frame = (state['t'], depth)
                                self.record_render_delivery('depth', state, started)
                    # 超时只降低实际频率；下一轮重新采集，不补发积压旧帧。
                    self.quit.wait(max(0.0, .125 - (time.monotonic() - started)))
        except Exception:
            self.render_error = '导航深度渲染失败\n' + traceback.format_exc()

    def render_ui(self):
        """主线程只交付冻结快照和缓存结果；OpenGL与JPEG在任务专属子进程执行。"""
        context = multiprocessing.get_context('spawn')
        parent, child = context.Pipe()
        stop = context.Event()
        process = context.Process(target=render_worker,
                                  args=(str(ROOT/'follow_demo/runtime/scene.xml'), child, stop, os.getpid()),
                                  name='go2-ui-render-worker', daemon=True)
        self.ui_process = process
        try:
            process.start()
            child.close()
            while not self.quit.is_set():
                if not parent.poll(.1):
                    if not process.is_alive():
                        raise RuntimeError('界面渲染子进程提前退出')
                    continue
                message = parent.recv()
                if message[0] == 'state':
                    with self.lock:
                        state = self.snapshot.copy()
                    parent.send(state)
                elif message[0] == 'frame':
                    _, name, state, started, frames, images, missed = message
                    with self.lock:
                        if state['epoch'] == self.epoch:
                            self.images.update({key:value for key,value in images.items() if value is not None})
                            if frames is not None:
                                self.frames = frames
                            self.record_render_delivery(name,state,started)
                            self.render_diagnostics[name]['skipped'] += missed
                            self.render_diagnostics['ui_process_pid'] = process.pid
                elif message[0] == 'error':
                    raise RuntimeError(message[1])
        except (EOFError, BrokenPipeError):
            if not self.quit.is_set():
                self.render_error = '界面渲染子进程通信中断'
        except Exception:
            self.render_error = '界面渲染失败\n'+traceback.format_exc()
        finally:
            stop.set()
            parent.close()
            child.close()
            if process.pid is not None:
                process.join(timeout=5.)
                if process.is_alive():
                    # 仅终止本对象创建的子进程；不会扫描或结束其他任务。
                    process.terminate()
                    process.join(timeout=2.)
                if process.is_alive():
                    self.render_error = '界面渲染子进程未退出'
