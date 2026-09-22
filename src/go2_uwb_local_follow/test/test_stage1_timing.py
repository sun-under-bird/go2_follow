# Copyright 2026 OpenAI
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Exercise actual ROS nodes with synthetic clock, odometry and delayed clouds."""

import math
import os
import signal
import struct
import subprocess
import tempfile
import time

import pytest

# All tests are local, isolated from the robot domain; no real command topic is used.
os.environ['ROS_DOMAIN_ID'] = str(150 + os.getpid() % 70)
os.environ['ROS_LOCALHOST_ONLY'] = '1'

from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import PointStamped, TwistStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402


def stamp(seconds):
    """Construct a ROS time without floating point nanosecond overflow."""
    from builtin_interfaces.msg import Time
    ns = round(seconds * 1e9)
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


def cloud(seconds, points, expiry=None):
    """Build xyz clouds, optionally carrying per-cell expiration metadata."""
    message = PointCloud2()
    message.header.stamp = stamp(seconds)
    message.header.frame_id = 'base_footprint'
    message.height = 1
    message.width = len(points)
    message.fields = [PointField(name=n, offset=i * 4, datatype=7, count=1)
                      for i, n in enumerate(('x', 'y', 'z'))]
    if expiry is not None:
        message.fields.append(PointField(name='expires_at', offset=16, datatype=8, count=1))
    message.point_step = 24 if expiry is not None else 12
    message.row_step = message.width * message.point_step
    message.data = b''.join(struct.pack('<fff4xd', x, y, 0.2, expiry[i]) if expiry is not None
                            else struct.pack('<fff', x, y, 0.2)
                            for i, (x, y) in enumerate(points))
    return message


class Rig:
    """Drive planner inputs with repeatable simulated source times."""

    def __init__(self, map_node=False, overrides=None):
        self.context = rclpy.Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('stage1_test_driver', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.clock_pub = self.node.create_publisher(Clock, '/clock', 10)
        self.odom_pub = self.node.create_publisher(Odometry, '/test/odom', 10)
        self.nominal_pub = self.node.create_publisher(TwistStamped, '/test/nominal', 10)
        self.target_pub = self.node.create_publisher(PointStamped, '/test/target', 10)
        self.cloud_pub = self.node.create_publisher(PointCloud2, '/test/cloud', 10)
        self.rows = []
        self.clouds = []
        self.time = 100.0
        self.origin = self.time
        self.vx = 0.0
        self.wz = 0.0
        self.map_node = map_node
        self.node.create_subscription(DiagnosticArray, '/go2_uwb_local_follow/control_cycle',
                                      self.receive, 100)
        self.node.create_subscription(PointCloud2,
                                      '/test/map' if map_node else
                                      '/go2_uwb_local_follow/control_obstacles',
                                      self.clouds.append, qos_profile_sensor_data)
        executable = os.environ['MAP_EXECUTABLE' if map_node else 'PLANNER_EXECUTABLE']
        parameters = {'use_sim_time': 'true', 'odom_topic': '/test/odom'}
        if map_node:
            parameters.update(input_observation_topic='/test/cloud',
                              output_obstacle_topic='/test/map')
        else:
            parameters.update(nominal_cmd_topic='/test/nominal', target_topic='/test/target',
                              obstacle_topic='/test/cloud', cmd_vel_topic='/test/cmd',
                              enable_motion='true', enable_self_filter='false',
                              emergency_confirm_frames='3', enable_emergency_reverse='false',
                              obstacle_timeout_sec='0.70')
        parameters.update(overrides or {})
        command = [executable, '--ros-args']
        for name, value in parameters.items():
            command += ['-p', f'{name}:={value}']
        self.log = tempfile.TemporaryFile()
        try:
            self.process = subprocess.Popen(command, stdout=self.log, stderr=self.log)
        except BaseException:
            self.log.close()
            self.executor.shutdown()
            self.node.destroy_node()
            self.context.shutdown()
            raise
        deadline = time.monotonic() + 8
        while self.cloud_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            self.spin(0.02)
        try:
            assert self.cloud_pub.get_subscription_count() > 0, 'node did not subscribe'
            self.pump(15)
        except BaseException:
            self.close()
            raise

    def receive(self, message):
        for status in message.status:
            self.rows.append(dict(state=status.message, **{v.key: v.value for v in status.values}))

    def spin(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            self.executor.spin_once(timeout_sec=0.002)

    def pose(self, seconds):
        t = seconds - self.origin
        return self.vx * t, self.wz * t

    def pump(self, count=12, make_cloud=None, odom_delay=0.0, jump=0.0):
        for _ in range(count):
            self.time += 0.02
            self.clock_pub.publish(Clock(clock=stamp(self.time)))
            self.spin(0.003)
            odom = Odometry()
            odom.header.stamp = stamp(self.time - odom_delay)
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            x, yaw = self.pose(self.time - odom_delay)
            odom.pose.pose.position.x = x + jump
            odom.pose.pose.orientation.z = math.sin(yaw / 2)
            odom.pose.pose.orientation.w = math.cos(yaw / 2)
            self.odom_pub.publish(odom)
            target = PointStamped()
            target.header.stamp = stamp(self.time)
            target.header.frame_id = 'base_footprint'
            target.point.x = 2.0
            self.target_pub.publish(target)
            nominal = TwistStamped()
            nominal.header = target.header
            nominal.twist.linear.x = 0.3
            self.nominal_pub.publish(nominal)
            message = make_cloud(self.time) if make_cloud else cloud(self.time - 0.08, [])
            if message is not None:
                self.cloud_pub.publish(message)
            self.spin(0.023)

    def close(self):
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=5)
        self.log.seek(0)
        output = self.log.read().decode(errors='replace')
        self.log.close()
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()
        assert self.process.returncode == 0, output


@pytest.fixture
def rig():
    """Create and clean up a planner in a private ROS domain."""
    instance = Rig()
    try:
        yield instance
    finally:
        instance.close()


def test_delayed_wall_is_compensated_each_cycle_after_spatial_crop(rig):
    """Recover a fixed wall under simultaneous translation and rotation."""
    rig.vx, rig.wz = 0.3, 0.4
    world_x, world_y = 3.02, 0.3

    def observation(t):
        source = t - 0.12
        x, yaw = rig.pose(source)
        return cloud(source, [(math.cos(yaw) * (world_x - x) + math.sin(yaw) * world_y,
                               -math.sin(yaw) * (world_x - x) + math.cos(yaw) * world_y)])

    rig.pump(20, observation)
    assert rig.clouds[-1].width == 1
    message = rig.clouds[-1]
    t = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
    x, yaw = rig.pose(t)
    px, py = struct.unpack_from('<ff', message.data)
    assert px == pytest.approx(math.cos(yaw) * (world_x - x) + math.sin(yaw) * world_y, abs=2e-5)
    assert py == pytest.approx(-math.sin(yaw) * (world_x - x) + math.cos(yaw) * world_y, abs=2e-5)


def test_old_future_zero_clouds_stop_and_valid_data_recovers(rig):
    """Fresh receipt cannot hide old source data or poison subsequent timestamps."""
    for make in (lambda t: cloud(t - 2, []), lambda t: cloud(t + 5, []), lambda t: cloud(0, []),
                 lambda t: cloud(-1, [])):
        # Start in a new clock epoch so an old packet is not just ignored behind a newer snapshot.
        rig.time -= 10
        rig.pump(12, make)
        assert float(rig.rows[-1]['published_v']) == 0
        assert rig.rows[-1]['state'] in ('SENSOR_TIMEOUT', 'OBSTACLE_INVALID', 'WAIT_OBSTACLE')
        rig.pump(15)
        assert rig.rows[-1]['state'] in ('PLANNING', 'RECOVERING')
        assert float(rig.rows[-1]['published_v']) > 0


def test_duplicate_source_frames_do_not_confirm_emergency(rig):
    """Repeated receipt of one occupied frame is one observation, not three."""
    frozen = cloud(rig.time + 0.02, [(0.50, 0.0)])
    rig.pump(16, lambda _: frozen)
    assert int(rig.rows[-1]['emergency_hit_count']) == 1
    assert rig.rows[-1]['emergency'] == 'false'
    rig.pump(8, lambda t: cloud(t, [(0.50, 0.0)]))
    assert rig.rows[-1]['emergency'] == 'true'
    assert float(rig.rows[-1]['published_v']) == 0


def test_expired_cells_are_not_refreshed_by_new_map_header(rig):
    """Each historical cell expires independently of map publication time."""
    rig.pump(12, lambda t: cloud(t - 0.08, [(1.0, 0.8), (2.0, 0.8)], [t - 0.01, t + 1]))
    assert rig.clouds[-1].width == 1
    assert struct.unpack_from('<f', rig.clouds[-1].data)[0] == pytest.approx(2.0)


def test_stale_odom_received_continuously_stops(rig):
    """Odometry freshness is based on its source timestamp as well as receipt."""
    rig.pump(18, odom_delay=1.0)
    assert rig.rows[-1]['state'] in ('ODOM_TIMEOUT', 'ODOM_INVALID')
    assert float(rig.rows[-1]['published_v']) == 0


def test_pose_jump_invalidates_cached_cloud_until_new_epoch_observation(rig):
    """Never transform an old map through a newly reset odometry origin."""
    frozen = cloud(rig.time - 0.04, [(2.0, 0.5)])
    rig.pump(10, lambda _: frozen, jump=2.0)
    assert rig.rows[-1]['state'] in ('OBSTACLE_INVALID', 'WAIT_OBSTACLE')
    assert float(rig.rows[-1]['published_v']) == 0
    rig.pump(15, jump=2.0)
    assert rig.rows[-1]['state'] in ('PLANNING', 'RECOVERING')


def test_empty_cloud_valid_but_malformed_cloud_stops(rig):
    """An invalid observation cannot masquerade as confirmed free space."""
    rig.pump(8)
    assert rig.rows[-1]['state'] in ('PLANNING', 'RECOVERING')

    def malformed(t):
        message = cloud(t, [(1, 0)])
        message.data = b''
        return message

    rig.pump(8, malformed)
    assert rig.rows[-1]['state'] == 'OBSTACLE_INVALID'
    assert float(rig.rows[-1]['published_v']) == 0


def test_map_preserves_cell_age_and_refuses_delayed_source_frames():
    """Map output retains source time and per-cell history across fresh empty frames."""
    instance = Rig(map_node=True)
    try:
        instance.pump(5, lambda t: cloud(t - 0.06, [(1.0, 0.3)]))
        seen = instance.time - 0.06
        instance.pump(8)
        message = instance.clouds[-1]
        assert message.width == 1
        fields = {f.name: f.offset for f in message.fields}
        last_seen = struct.unpack_from('<d', message.data, fields['last_seen'])[0]
        assert last_seen == pytest.approx(seen)
        expiry = struct.unpack_from('<d', message.data, fields['expires_at'])[0]
        assert expiry == pytest.approx(seen + 1.0)
        count = len(instance.clouds)
        instance.pump(10, lambda t: cloud(t - 2, []))
        assert len(instance.clouds) == count
    finally:
        instance.close()


def test_held_snapshot_is_retransformed_without_accumulated_drift(rig):
    """A cached observation keeps its world location while the robot keeps moving."""
    rig.vx = 0.2
    rig.pump(10)
    source = rig.time + 0.02
    source_x, _ = rig.pose(source)
    frozen = cloud(source, [(2.0 - source_x, 0.5)])
    rig.pump(12, lambda _: frozen)
    message = rig.clouds[-1]
    t = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
    x, _ = rig.pose(t)
    assert struct.unpack_from('<f', message.data)[0] == pytest.approx(2.0 - x, abs=2e-5)


def test_pose_lookup_does_not_bridge_odom_dropout(rig):
    """A fresh odom endpoint does not make the intervening missing poses known."""
    rig.time += 1.0
    rig.pump(3)
    assert rig.rows[-1]['state'] == 'OBSTACLE_ALIGNMENT_FAILED'
    assert float(rig.rows[-1]['published_v']) == 0
    rig.pump(12)
    assert rig.rows[-1]['state'] in ('PLANNING', 'RECOVERING')


def test_reverse_footprint_rear_is_not_cropped():
    """Retain obstacles beyond the old -0.75 m cutoff when the reverse sweep needs them."""
    instance = Rig(overrides={'enable_emergency_reverse': 'true',
                              'emergency_reverse_distance': '0.40'})
    try:
        instance.pump(8, lambda t: cloud(t - 0.08, [(-0.80, 0.0)]))
        assert instance.clouds[-1].width == 1
        assert struct.unpack_from('<f', instance.clouds[-1].data)[0] == pytest.approx(-0.80)
    finally:
        instance.close()


def test_legacy_timing_switches_preserve_source_frame_for_comparison():
    """Verify the receipt-time/no-compensation baseline switches."""
    instance = Rig(overrides={'compensate_obstacle_motion': 'false',
                              'enforce_source_time': 'false'})
    try:
        instance.time += 3.0
        instance.vx = 0.3
        instance.pump(12, lambda t: cloud(t - 2.0, [(1.0, 1.0)]))
        assert float(instance.rows[-1]['obstacle_source_age_sec']) >= 2.0
        assert float(instance.rows[-1]['published_v']) > 0
        assert struct.unpack_from('<f', instance.clouds[-1].data)[0] == pytest.approx(1.0)
    finally:
        instance.close()
