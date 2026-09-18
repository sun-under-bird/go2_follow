# Copyright 2026 OpenAI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""通过伪终端验证真实驱动线程在设备缺席、拔插和半帧中断后恢复发布."""

import os
import pty
import struct
import subprocess
import sys
import tempfile
import time

import rclpy
from uwb_aoa_pkg.msg import LibAoaRobotMsg


def make_frame(distance):
    """按照 C5 协议生成距离可区分的新数据帧."""
    payload = bytearray(38)
    struct.pack_into('<fff', payload, 14, distance, 0.0, 0.0)
    data = bytes([0xC5, len(payload)]) + payload
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return bytes([0x55, 0xAA, 1, len(data), 0]) + data + struct.pack('>H', crc)


def run_test(driver):
    """先在设备不存在时启动，再两次连接伪串口，确认无需重启节点."""
    rclpy.init(args=[])
    node = rclpy.create_node('serial_reconnect_test')
    received = []

    def record(message):
        """记录驱动发布的距离，区分重连前后样本."""
        received.append(message.x)

    node.create_subscription(LibAoaRobotMsg, '/serial_test/raw', record, 10)
    master = slave = None
    process = None
    with tempfile.TemporaryDirectory() as directory:
        device = os.path.join(directory, 'uwb')
        log_path = os.path.join(directory, 'driver.log')
        with open(log_path, 'w') as log:
            try:
                process = subprocess.Popen(
                    [driver, device, '--ros-args', '-r',
                     '/libAoa_robot_publisher:=/serial_test/raw'],
                    stdout=log, stderr=subprocess.STDOUT)
                time.sleep(0.25)
                assert process.poll() is None, '设备不存在时驱动提前退出'
                for distance in (2.0, 3.0):
                    master, slave = pty.openpty()
                    if os.path.lexists(device):
                        os.unlink(device)
                    os.symlink(os.ttyname(slave), device)
                    received.clear()
                    deadline = time.monotonic() + 5.0
                    while distance not in received and time.monotonic() < deadline:
                        os.write(master, make_frame(distance))
                        rclpy.spin_once(node, timeout_sec=0.05)
                        time.sleep(0.05)
                    assert distance in received, '驱动未能自动连接或重新连接'
                    # 半帧沉默后新帧必须恢复，且驱动进程保持存活。
                    os.write(master, make_frame(distance)[:12])
                    time.sleep(0.3)
                    os.close(master)
                    os.close(slave)
                    master = slave = None
                    time.sleep(0.3)
                    assert process.poll() is None, '拔出串口后驱动退出'
            except Exception:
                with open(log_path) as output:
                    print(output.read())
                raise
            finally:
                if master is not None:
                    os.close(master)
                if slave is not None:
                    os.close(slave)
                if process is not None:
                    process.terminate()
                    process.wait(timeout=5)
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    run_test(sys.argv[1])

