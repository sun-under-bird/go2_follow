"""回环域 143 的合成消息验收：无操作消息启动，分别断开真实输入接口。"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np


def main():
    package = Path(__file__).resolve().parents[1]
    evidence = package.parent / 'artifacts/hardware-adaptation'
    evidence.mkdir(parents=True, exist_ok=True)
    # 测试主动限定为回环网络，父子进程均不连接实机 DDS 网络。
    os.environ['ROS_DOMAIN_ID'] = '143'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    os.environ['CYCLONEDDS_URI'] = (package / 'cyclonedds.xml').as_uri()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PointStamped, TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    from lifecycle_msgs.srv import GetState
    from follow_demo.mppi_runtime import MANIFEST, ROOT

    if MANIFEST.exists():
        raise RuntimeError('请先结束已有跟随运行进程，再执行独立验收')
    config = ROOT / 'follow_demo/runtime/mppi.yaml'
    previous = config.read_bytes() if config.exists() else None
    rclpy.init(args=[])
    node = Node('hardware_automatic_fixture')
    process = None
    log = (evidence / 'automatic-runtime.log').open('w')
    try:
        tf = TransformBroadcaster(node)
        static_tf = StaticTransformBroadcaster(node)
        camera = TransformStamped()
        camera.header.stamp = node.get_clock().now().to_msg()
        camera.header.frame_id, camera.child_frame_id = 'base_footprint', 'fixture_depth_optical'
        # 合成俯视相机使起点包络确实可见；不使用人工净空填图。
        camera.transform.translation.z = .435
        camera.transform.rotation.x = 1.
        camera.transform.rotation.w = 0.
        static_tf.sendTransform(camera)
        pub = {}
        for name, kind, topic in (('odom', Odometry, '/odom'), ('target', PointStamped, '/uwb/target_point'),
                                 ('depth', Image, '/stereo/depth_debug'),
                                 ('info', CameraInfo, '/camera/camera/infra1/camera_info'),
                                 ('cloud', PointCloud2, '/local_grid_obstacle')):
            pub[name] = node.create_publisher(kind, topic, qos_profile_sensor_data)
        counts, states, commands = {}, [], []
        start = time.monotonic()

        def record(name, message):
            counts[name] = counts.get(name, 0) + 1
            elapsed = time.monotonic() - start
            if name == 'status':
                states.append((elapsed, json.loads(message.data)))
            if name == 'command':
                commands.append((elapsed, message.linear.x, message.angular.z))

        subscriptions = []
        for name, kind, topic, qos in (('command', Twist, '/cmd_vel', 5),
                                      ('status', String, '/go2_follow/control_status', 5),
                                      ('depth', Image, '/go2_follow/input/depth', qos_profile_sensor_data),
                                      ('cloud', PointCloud2, '/go2_follow/input/obstacles', qos_profile_sensor_data)):
            subscriptions.append(node.create_subscription(kind, topic, lambda message, name=name: record(name, message), qos))
        service = node.create_client(GetState, '/go2_follow_mppi/controller_server/get_state')
        state_future = None
        operator_subscribers = None

        def publish():
            elapsed = time.monotonic() - start
            stamp = node.get_clock().now().to_msg()
            if not 8 <= elapsed < 9.5:
                odom = Odometry()
                odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_footprint'
                odom.pose.pose.orientation.w = 1.
                pub['odom'].publish(odom)
                body = TransformStamped()
                body.header.stamp, body.header.frame_id, body.child_frame_id = stamp, 'odom', 'base_footprint'
                body.transform.rotation.w = 1.
                tf.sendTransform(body)
            if not 5 <= elapsed < 6.5:
                target = PointStamped()
                target.header.stamp, target.header.frame_id, target.point.x = stamp, 'base_footprint', 4.
                pub['target'].publish(target)
            info = CameraInfo()
            info.header.stamp, info.header.frame_id = stamp, 'fixture_depth_optical'
            info.width, info.height = 64, 48
            info.k = [5., 0., 31.5, 0., 5., 23.5, 0., 0., 1.]
            pub['info'].publish(info)
            if not 14 <= elapsed < 15.5:
                depth = Image()
                depth.header = info.header
                depth.width, depth.height, depth.encoding, depth.step = 64, 48, '32FC1', 256
                depth.data = np.full((48, 64), .435, dtype='<f4').tobytes()
                pub['depth'].publish(depth)
            if not 11 <= elapsed < 12.5:
                cloud = PointCloud2()
                cloud.header.stamp, cloud.header.frame_id = stamp, 'base_footprint'
                cloud.height, cloud.width, cloud.point_step, cloud.row_step = 1, 1, 12, 12
                cloud.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                                for i, name in enumerate(('x', 'y', 'z'))]
                cloud.data = np.array([[1.8, 1., .3]], dtype='<f4').tobytes()
                pub['cloud'].publish(cloud)

        timer = node.create_timer(.05, publish)
        process = subprocess.Popen([sys.executable, '-m', 'follow_demo.hardware_runtime', '--depth-frame',
                                    'fixture_depth_optical', '--duration', '18'], stdout=log, stderr=subprocess.STDOUT)
        while process.poll() is None and time.monotonic() - start < 30:
            rclpy.spin_once(node, timeout_sec=.05)
            if state_future is None and time.monotonic() - start > 4 and service.service_is_ready():
                state_future = service.call_async(GetState.Request())
            if time.monotonic() - start > 16:
                operator_subscribers = len(node.get_subscriptions_info_by_topic('/go2_follow/operator'))
        if process.poll() is None:
            process.terminate()
        code = process.wait(timeout=10)

        def stopped(begin, end, reason):
            return any(begin < elapsed < end and status['navigation']['code'] == reason and status['command'] == [0., 0.]
                       for elapsed, status in states)

        result = dict(runtime_exit=code, command_topic='/cmd_vel', counts=counts,
                      initial_reasons=sorted({status['navigation']['code'] for elapsed, status in states if elapsed < 5}),
                      nav2_active=bool(state_future and state_future.done() and state_future.result().current_state.id == 3),
                      operator_messages_sent=0, operator_subscribers=operator_subscribers,
                      automatic_motion=any(elapsed < 5 and vx > .01 for elapsed, vx, wz in commands),
                      no_manual_gate=bool(states) and all(status['navigation']['code'] not in
                          ('OPERATOR_STALE', 'START_UNCONFIRMED', 'PAUSED', 'NOT_READY') for elapsed, status in states),
                      automatic_mode=bool(states) and all(status['operation_mode'] == 'automatic' for elapsed, status in states),
                      no_artificial_clearance=bool(states) and all(not status['navigation']['map']['confirmed'] for elapsed, status in states),
                      target_loss_stops=stopped(5.4, 6.5, 'TARGET_INVALID'),
                      odom_loss_stops=stopped(8.15, 9.5, 'ODOM_STALE'),
                      cloud_loss_stops=stopped(11.8, 12.5, 'MAP_STALE'),
                      depth_loss_stops=stopped(14.8, 15.5, 'MAP_STALE'),
                      test_scope='Synthetic visible ground on loopback domain 143; no physical hardware')
        checks = ('nav2_active', 'automatic_motion', 'no_manual_gate', 'automatic_mode', 'no_artificial_clearance',
                  'target_loss_stops', 'odom_loss_stops', 'cloud_loss_stops', 'depth_loss_stops')
        result['passed'] = code == 0 and operator_subscribers == 0 and all(result[key] for key in checks)
        (evidence / 'automatic-ros-check.json').write_text(json.dumps(result, indent=2))
        (evidence / 'automatic-status.json').write_text(json.dumps(states))
        print(json.dumps(result))
        if not result['passed']:
            raise RuntimeError('automatic hardware ROS integration failed')
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait(timeout=10)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        if previous is None:
            config.unlink(missing_ok=True)
        else:
            config.write_bytes(previous)


if __name__ == '__main__':
    main()
