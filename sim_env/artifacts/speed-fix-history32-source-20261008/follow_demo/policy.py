"""复用已锁定 ONNX 模型的观测、历史栈、动作滤波和 PD 力矩执行语义。"""
import math
import numpy as np
import onnxruntime as ort
from .controller import wrap
from .yaw_rate import YawRateServo
from .forward_speed import ForwardSpeedServo


DEFAULT = np.array([-0.1, 0.8, -1.5, 0.1, 0.8, -1.5,
                    -0.1, 1.0, -1.5, 0.1, 1.0, -1.5])


def yaw_from_quaternion(quaternion):
    """从 MuJoCo 的 wxyz 四元数提取世界系偏航角。"""
    w, x, y, z = quaternion
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class Policy:
    """以 50 Hz 计算关节目标，并在每个物理步计算电机力矩。"""
    def __init__(self, model, data, path, yaw_mode='heading'):
        """载入锁定模型，按 SDK 的 FR、FL、RR、RL 顺序读取具名传感器。"""
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        self.session = ort.InferenceSession(str(path), options, providers=['CPUExecutionProvider'])
        self.model = model
        self.base_id = model.body('base_link').id
        if yaw_mode not in ('velocity', 'heading', 'rate'):
            raise ValueError('未知的转向命令语义')
        self.yaw_mode = yaw_mode
        self.rate_servo = YawRateServo()
        self.forward_servo = ForwardSpeedServo()
        self.names = [f'{leg}_{joint}' for leg in ('FR', 'FL', 'RR', 'RL') for joint in ('hip', 'thigh', 'calf')]
        assert [model.actuator(index).name for index in range(model.nu)] == self.names
        self.reset(data)
        self.session.run(None, {'obs': np.zeros((1, 45), np.float32), 'hist': np.zeros((1, 10, 45), np.float32)})

    def positions(self, data):
        """读取 SDK 顺序关节角，避免直接把 MuJoCo qpos 的 FL 开头顺序当作 SDK 顺序。"""
        return np.array([data.sensor(name + '_pos').data[0] for name in self.names])

    def velocities(self, data):
        """读取 SDK 顺序的关节角速度。"""
        return np.array([data.sensor(name + '_vel').data[0] for name in self.names])

    def reset(self, data):
        """清空上一轮历史和动作，起立过渡从当前物理关节姿态开始。"""
        self.initial = self.positions(data)
        self.target = self.initial.copy()
        self.history = np.zeros((1, 10, 45), np.float32)
        self.last_action = np.zeros(12)
        self.filtered_action = np.zeros(12)
        self.initialized = False
        self.next_update = 0.0
        self.heading = yaw_from_quaternion(data.sensor('imu_quat').data)
        self.last_yaw = self.heading
        self.last_command = np.zeros(3)
        self.last_policy_wz = 0.0
        self.rate_servo.reset()
        self.forward_servo.reset()
        self.kp = 20.0
        self.inference_count = 0

    def update(self, data, command):
        """展开上游策略的同一观测布局；本版不补偿或抬升低速命令。"""
        time = data.time
        if time + 1e-8 < self.next_update:
            return
        self.next_update = time + 0.02
        if time < 3.0:
            phase = math.tanh(time / 1.2)
            self.target = self.initial * (1 - phase) + DEFAULT * phase
            self.kp = 20 + phase * 20
            self.heading = yaw_from_quaternion(data.sensor('imu_quat').data)
            return
        self.kp = 40.0
        vx, vy, wz = command
        quaternion = data.sensor('imu_quat').data
        yaw = yaw_from_quaternion(quaternion)
        self.last_yaw = yaw
        self.last_command = np.asarray(command, dtype=float).copy()
        if abs(vx) + abs(vy) + abs(wz) < 1e-6:
            # 零速度不保留先前积累的转向目标，避免停止后继续执行旧转向。
            self.heading = yaw
            self.rate_servo.reset()
        elif self.yaw_mode == 'rate':
            # rate 不积累另一个航向目标，避免把未使用的旧航向误差误读为控制状态。
            self.heading = yaw
        else:
            self.heading = wrap(self.heading + wz * 0.02)
        # 默认复用锁定上游的航向误差外环；模型的第三个命令仍是角速度。
        # 恒定转向基准说明外环可持续转向，不能据此证明闭环快速换向和跟随效果。
        # velocity 保留用于独立闭环对照，各模式均不增加最低速度、死区补偿或起步淡入。
        if self.yaw_mode == 'rate':
            # rate 的上层接口是期望机身角速度，闭环只修改模型转向输入。
            # 全零是停车请求，清旧积分后输出零；实际停车仍由物理反馈与安全层核对。
            policy_wz = (0. if abs(vx)+abs(vy)+abs(wz) < 1e-6 else
                         self.rate_servo.update(vx,wz,float(data.sensor('imu_gyro').data[2]),.02))
        else:
            policy_wz = np.clip(wz if self.yaw_mode == 'velocity' else 0.5 * wrap(self.heading - yaw), -1.0, 1.0)
        self.last_policy_wz = float(policy_wz)
        # 底盘请求仍是0.8上限；策略内部输入单独标定，绝不修改物理位置或速度。
        rotation = data.xmat[self.base_id].reshape(3, 3)
        measured_vx = float((rotation.T @ data.sensor('frame_vel').data)[0])
        policy_vx = self.forward_servo.update(float(vx), measured_vx)
        w, x, y, z = quaternion
        gravity = np.array([2 * (w * y - x * z), -2 * (y * z + w * x), 2 * (x * x + y * y) - 1])
        observation = np.concatenate((data.sensor('imu_gyro').data * 0.25, gravity,
                                      [policy_vx * 2, vy * 2, policy_wz * 0.25],
                                      self.positions(data) - DEFAULT,
                                      self.velocities(data) * 0.05, self.last_action)).astype(np.float32)
        action = self.session.run(None, {'obs': observation.reshape(1, 45), 'hist': self.history})[0].reshape(12)
        if not np.isfinite(action).all():
            raise RuntimeError('ONNX 返回非有限动作')
        self.filtered_action = 0.8 * action + 0.2 * self.filtered_action
        scale = np.full(12, 0.25)
        scale[::3] *= 0.5
        self.target = DEFAULT + scale * self.filtered_action
        if not self.initialized:
            self.history[:] = observation
            self.initialized = True
        else:
            self.history[:, :-1] = self.history[:, 1:].copy()
            self.history[:, -1] = observation
        self.last_action = action.copy()
        self.inference_count += 1

    def execution_diagnostics(self):
        """记录所选实验模式、请求角速度和送入模型的角速度，供闭环 A/B 核对。"""
        result = dict(mode=self.yaw_mode, requested_command=self.last_command.tolist(),
                    forward_servo=dict(self.forward_servo.last),
                    policy_wz=self.last_policy_wz, heading=self.heading,
                    heading_error=wrap(self.heading - self.last_yaw))
        if self.yaw_mode == 'rate':
            result.update(rate_servo=dict(self.rate_servo.last), rate_configuration=self.rate_servo.configuration())
        return result

    def torque(self, data):
        """每个 MuJoCo 物理步重新计算 PD 力矩，并遵守模型电机力矩上限。"""
        torque = self.kp * (self.target - self.positions(data)) - self.velocities(data)
        data.ctrl[:] = np.clip(torque, self.model.actuator_ctrlrange[:, 0], self.model.actuator_ctrlrange[:, 1])
