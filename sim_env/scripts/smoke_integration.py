"""实际启动 MuJoCo、策略和 CAPO，验证跨 SDK/ROS 的状态链路。"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import LowState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

ROOT = Path.home() / 'go2_sim'


def launch_process(command, label, processes):
    """启动本次检查拥有的进程组，分别保存日志，方便定位链路故障。"""
    stream = (ROOT / 'logs' / (label + '.log')).open('w')
    process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append((process, stream))
    return process


def stop_processes(processes):
    """停止本次启动的进程，不影响其他 WSL/ROS 工作。"""
    for process, stream in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        stream.close()


def main():
    """在隔离域中检查真实仿真数据是否能到达 CAPO 输出，运行后清理进程。"""
    assert os.environ.get('ROS_DOMAIN_ID') == '1', '必须先加载 go2_sim/setup.bash'
    processes = []
    counts = {'lowstate': 0, 'imu': 0, 'odom': 0}
    invalid_odom = 0
    last_position = None
    rclpy.init()
    node = rclpy.create_node('go2_native_integration_check')

    def on_lowstate(message):
        """统计来自原生模拟器的 LowState。"""
        counts['lowstate'] += 1

    def on_imu(message):
        """统计 LowState 适配后的相机以外的机体 IMU 消息。"""
        counts['imu'] += 1

    def on_odom(message):
        """检查 CAPO 输出存在且数值有限；不将此检查解释为精度验证。"""
        nonlocal invalid_odom, last_position
        counts['odom'] += 1
        position = message.pose.pose.position
        last_position = [position.x, position.y, position.z]
        orientation = message.pose.pose.orientation
        values = last_position + [orientation.x, orientation.y, orientation.z, orientation.w]
        invalid_odom += int(not all(math.isfinite(value) for value in values))

    subscriptions = [
        node.create_subscription(LowState, '/lowstate', on_lowstate, qos_profile_sensor_data),
        node.create_subscription(Imu, '/SMX/Go2IMU', on_imu, qos_profile_sensor_data),
        node.create_subscription(Odometry, '/SMX/Odom', on_odom, qos_profile_sensor_data),
    ]
    report = {'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'duration_wall_s': 25}
    try:
        # 使用 WSLg 实际图形窗口，与用户日常启动使用同一可执行程序。
        simulator = launch_process([str(ROOT / 'bin/go2-sim')], 'native-simulator', processes)
        estimator = launch_process([str(ROOT / 'bin/go2-capo')], 'native-capo', processes)
        start = time.monotonic()
        policy = None
        while time.monotonic() - start < 25:
            rclpy.spin_once(node, timeout_sec=0.05)
            # 确认模拟器已经开始发送状态，再启动站立策略。
            if policy is None and counts['lowstate'] > 20:
                policy = launch_process([str(ROOT / 'bin/go2-policy'), '30', '0', '0', '0'],
                                        'native-policy', processes)
            if simulator.poll() is not None or estimator.poll() is not None:
                raise RuntimeError('模拟器或 CAPO 提前退出，请检查对应日志')
        report.update(counts=counts, invalid_odom=invalid_odom, last_position=last_position,
                      policy_running=policy is not None and policy.poll() is None)
        report['passed'] = (all(count > 50 for count in counts.values()) and invalid_odom == 0
                            and report['policy_running'])
    except Exception as error:
        report.update(passed=False, error=str(error), counts=counts)
    finally:
        # 先停模拟器，避免退出策略后残留命令继续推进；之后按组清理其他进程。
        if processes and processes[0][0].poll() is None:
            os.killpg(processes[0][0].pid, signal.SIGINT)
        stop_processes(processes)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        path = ROOT / 'artifacts/native-integration-check.json'
        path.write_text(json.dumps(report, indent=2, ensure_ascii=False))
        print(json.dumps(report, indent=2, ensure_ascii=False))
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
