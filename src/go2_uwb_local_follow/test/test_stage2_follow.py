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

"""Stage two end-to-end tests in an isolated ROS domain with simulated source time."""

import math
import os
import signal
import subprocess
import tempfile
import time

from test_stage1_timing import cloud, Rig, stamp

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import pytest
from rosgraph_msgs.msg import Clock
from uwb_aoa_pkg.msg import LibAoaRobotMsg


class FollowRig(Rig):
    """Connect the actual controller and planner, never to the robot command topic."""

    def __init__(self, enabled=True):
        self.children = []
        self.enabled = enabled
        self.states = []
        self.follow_rows = []
        self.world = lambda t: (2 + .4 * t, 0.0)
        self.delay = .10
        super().__init__(overrides={'use_target_state': str(enabled).lower(),
                                    'target_state_topic': '/test/state'})
        self.node.create_subscription(Odometry, '/test/state', self.states.append, 100)
        self.node.create_subscription(DiagnosticArray, '/go2_uwb_local_follow/follow_cycle',
                                      self.receive_follow, 100)
        self.start('FOLLOW_EXECUTABLE', {
            'use_sim_time': 'true', 'odom_topic': '/test/odom',
            'target_topic': '/test/target', 'target_state_topic': '/test/state',
            'nominal_cmd_topic': '/test/nominal', 'cmd_vel_topic': '/test/follow_cmd',
            'enable_motion': 'false', 'enable_target_estimation': str(enabled).lower()})
        deadline = time.monotonic() + 8
        while self.target_pub.get_subscription_count() < 2 and time.monotonic() < deadline:
            self.spin(.02)
        assert self.target_pub.get_subscription_count() == 2
        self.drive(30)

    def start(self, env, parameters):
        command = [os.environ[env], '--ros-args']
        for key, value in parameters.items():
            command += ['-p', f'{key}:={value}']
        log = tempfile.TemporaryFile()
        self.children.append((subprocess.Popen(command, stdout=log, stderr=log), log))

    def receive_follow(self, msg):
        for status in msg.status:
            self.follow_rows.append(dict(state=status.message,
                                         **{v.key: v.value for v in status.values}))

    def drive(self, count, target_stamp=None, offset=0.0, jump=0.0, obstacle=False,
              odom_delay=0.0):
        for _ in range(count):
            self.time += .05
            self.clock_pub.publish(Clock(clock=stamp(self.time)))
            self.spin(.004)
            odom = Odometry()
            odom.header.stamp = stamp(self.time - odom_delay)
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            x, yaw = self.pose(self.time - odom_delay)
            odom.pose.pose.position.x = x + jump
            odom.pose.pose.orientation.z = math.sin(yaw / 2)
            odom.pose.pose.orientation.w = math.cos(yaw / 2)
            self.odom_pub.publish(odom)
            self.spin(.004)
            source = self.time - self.delay if target_stamp is None else target_stamp
            hx, hy = self.world(source - self.origin)
            x, yaw = self.pose(source)
            dx, dy = hx - x, hy
            target = PointStamped()
            target.header.stamp = stamp(source)
            target.header.frame_id = 'base_footprint'
            target.point.x = math.cos(yaw)*dx + math.sin(yaw)*dy + offset
            target.point.y = -math.sin(yaw)*dx + math.cos(yaw)*dy
            self.target_pub.publish(target)
            self.cloud_pub.publish(cloud(self.time, [(0.45, 0.0)] if obstacle else []))
            self.spin(.055)

    def close(self):
        errors = []
        for process, log in self.children:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            log.seek(0)
            if process.returncode != 0:
                errors.append(log.read().decode(errors='replace'))
            log.close()
        super().close()
        assert not errors, errors


@pytest.fixture
def follow():
    rig = FollowRig()
    try:
        yield rig
    finally:
        rig.close()


def test_source_time_world_velocity_and_current_geometry(follow):
    follow.vx, follow.wz = .25, .15
    # Changing the synthetic odometry motion model creates a jump: allow fresh warmup.
    follow.drive(35)
    e = follow.states[-1]
    source = e.header.stamp.sec + e.header.stamp.nanosec * 1e-9
    assert source == pytest.approx(follow.time - follow.delay, abs=1e-6)
    assert e.header.frame_id == e.child_frame_id == 'odom'
    assert e.pose.pose.position.x == pytest.approx(2 + .4*(source-follow.origin), abs=.02)
    assert e.twist.twist.linear.x == pytest.approx(.4, abs=.02)
    assert abs(e.twist.twist.linear.y) < .02
    row = follow.rows[-1]
    assert row['use_target_state'] == 'true'
    assert float(row['distance']) == pytest.approx(
        2 + (.4-.25)*(follow.time-follow.origin), abs=.04)


def test_jump_and_duplicate_do_not_refresh_or_steer(follow):
    before = follow.states[-1]
    follow.drive(1, offset=5)
    assert follow.states[-1].header.stamp == before.header.stamp
    assert follow.follow_rows[-1]['target_update'] == 'OUTLIER'
    frozen = before.header.stamp.sec + before.header.stamp.nanosec*1e-9
    follow.drive(14, target_stamp=frozen)
    assert follow.states[-1].header.stamp == before.header.stamp
    assert follow.follow_rows[-1]['state'] == 'TARGET_LOST'
    assert follow.rows[-1]['state'] == 'TARGET_INVALID_OR_TIMEOUT'
    follow.drive(12)
    assert follow.follow_rows[-1]['have_result'] == 'true'


def test_old_and_future_source_packets_are_rejected(follow):
    before = follow.states[-1].header.stamp
    follow.drive(12, target_stamp=follow.time - 5)
    assert follow.states[-1].header.stamp == before
    assert follow.follow_rows[-1]['state'] == 'TARGET_LOST'
    follow.drive(3, target_stamp=follow.time + 5)
    assert follow.states[-1].header.stamp == before
    follow.drive(12)
    assert follow.follow_rows[-1]['have_result'] == 'true'


def test_clock_reset_and_odom_reset_need_new_estimates(follow):
    follow.drive(1, jump=3)
    assert follow.follow_rows[-1]['state'] == 'WAIT_TARGET'
    follow.drive(12, jump=3)
    assert follow.follow_rows[-1]['have_result'] == 'true'
    follow.time -= 20
    follow.drive(1)
    assert follow.follow_rows[-1]['state'] == 'WAIT_TARGET'
    follow.drive(14)
    assert follow.follow_rows[-1]['have_result'] == 'true'


def test_sudden_obstacle_still_stops_planner(follow):
    assert float(follow.follow_rows[-1]['nominal_v']) > .23
    follow.drive(8, obstacle=True)
    assert follow.rows[-1]['emergency'] == 'true', follow.rows[-1]
    assert float(follow.rows[-1]['published_v']) == 0


def test_rollback_uses_raw_distance_following():
    rig = FollowRig(enabled=False)
    try:
        assert not rig.states
        assert rig.follow_rows[-1]['enable_target_estimation'] == 'false'
        assert rig.rows[-1]['use_target_state'] == 'false'
        assert float(rig.follow_rows[-1]['nominal_v']) > .23
    finally:
        rig.close()


def test_adapter_preserves_source_stamp(follow):
    points = []
    pub = follow.node.create_publisher(LibAoaRobotMsg, '/test/raw', 10)
    follow.node.create_subscription(PointStamped, '/test/adapted', points.append, 10)
    follow.start('ADAPTER_EXECUTABLE', {
        'use_sim_time': 'true', 'raw_topic': '/test/raw',
        'target_topic': '/test/adapted'})
    deadline = time.monotonic() + 8
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        follow.spin(.02)
    message = LibAoaRobotMsg()
    message.header.stamp = stamp(follow.time-.3)
    message.x = 2.0
    pub.publish(message)
    follow.spin(.15)
    assert points[-1].header.stamp == message.header.stamp
    assert points[-1].point.x == 2.0


def test_expired_odometry_stops_controller_and_planner(follow):
    follow.drive(12, odom_delay=.5)
    # Missing odom also prevents new world targets; either watchdog can stop first.
    assert follow.follow_rows[-1]['state'] in ('WAIT_ODOM', 'ODOM_TIMEOUT', 'TARGET_LOST')
    assert follow.follow_rows[-1]['have_result'] == 'false'
    assert float(follow.rows[-1]['published_v']) == 0
    follow.drive(14)
    assert follow.follow_rows[-1]['have_result'] == 'true'
