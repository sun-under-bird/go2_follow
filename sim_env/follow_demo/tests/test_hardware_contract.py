"""真实输入的单位、帧和会话边界，以及算法参数保持一致的回归。"""
import copy
import math
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np

from follow_demo.hardware_contract import cloud_xyz, depth_in_meters, mark_obstacles, transform_point, transform_points, validate_odometry
from follow_demo.mppi_runtime import MppiRuntime, parameters
from follow_demo.ros_contract import RosInterface, hardware_interface, yaw_from_quaternion
from follow_demo.local_map import RollingMap


def vector(x=0., y=0., z=0.):
    return NS(x=x, y=y, z=z)


def odometry():
    return NS(header=NS(frame_id='odom', stamp=NS(sec=1, nanosec=50)), child_frame_id='base_footprint',
              pose=NS(pose=NS(position=vector(), orientation=NS(w=1., x=0., y=0., z=0.))),
              twist=NS(twist=NS(linear=vector(.2), angular=vector(z=.1))))


class HardwareContractTests(unittest.TestCase):
    def test_padded_integer_depth_preserves_invalid_samples_and_input(self):
        raw = np.array([[1000, 0, 9999], [2000, 3000, 9999]], dtype='<u2').tobytes()
        output = depth_in_meters(raw, 2, 2, 6, '16UC1')
        np.testing.assert_allclose(output, [[1., np.nan], [2., 3.]], equal_nan=True)
        self.assertEqual(len(raw), 12)
        self.assertTrue(output.flags.c_contiguous)

    def test_actual_driver_scale_is_explicit(self):
        output = depth_in_meters(np.array([2000], dtype='>u2').tobytes(), 1, 1, 2, '16UC1', True, .0005)
        self.assertEqual(output[0, 0], 1.)

    def test_float_meters_are_not_scaled_and_bad_depth_stays_unknown(self):
        raw = np.array([[1.5, -1., np.inf, np.nan]], dtype='>f4').tobytes()
        output = depth_in_meters(raw, 4, 1, 16, '32FC1', True)
        np.testing.assert_allclose(output, [[1.5, np.nan, np.nan, np.nan]], equal_nan=True)

    def test_malformed_depth_cannot_be_accepted(self):
        for arguments in ((b'00', 2, 1, 2, '16UC1'), (b'000', 1, 1, 3, '16UC1'),
                          (b'0', 1, 1, 2, '16UC1'), (b'0000', 1, 1, 4, 'rgb8')):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                depth_in_meters(*arguments)
        with self.assertRaises(ValueError):
            depth_in_meters(b'00', 1, 1, 2, '16UC1', integer_scale=0.)

    def test_pose_velocity_frames_and_stamp_are_required(self):
        valid = odometry()
        validate_odometry(valid, 'odom', 'base_footprint')
        invalid = copy.deepcopy(valid)
        invalid.child_frame_id = 'base_link'
        with self.assertRaises(ValueError):
            validate_odometry(invalid, 'odom', 'base_footprint')
        invalid = copy.deepcopy(valid)
        invalid.twist.twist.angular.z = math.nan
        with self.assertRaises(ValueError):
            validate_odometry(invalid, 'odom', 'base_footprint')
        invalid = copy.deepcopy(valid)
        invalid.header.stamp.sec = 0
        invalid.header.stamp.nanosec = 0
        with self.assertRaises(ValueError):
            validate_odometry(invalid, 'odom', 'base_footprint')
        invalid = copy.deepcopy(valid)
        invalid.pose.pose.orientation.w = 0.
        with self.assertRaises(ValueError):
            validate_odometry(invalid, 'odom', 'base_footprint')

    def test_target_transform_rotates_and_translates(self):
        tf = NS(rotation=NS(w=math.sqrt(.5), x=0., y=0., z=math.sqrt(.5)), translation=vector(2., 3., 4.))
        np.testing.assert_allclose(transform_point(vector(1.), tf), [2., 4., 4.], atol=1e-12)
        self.assertAlmostEqual(yaw_from_quaternion([tf.rotation.w, 0., 0., tf.rotation.z]), math.pi / 2)

    def test_invalid_transform_does_not_only_relabel_target(self):
        tf = NS(rotation=NS(w=0., x=0., y=0., z=0.), translation=vector())
        with self.assertRaises(ValueError):
            transform_point(vector(1.), tf)

    def test_cloud_padding_and_endianness(self):
        fields = [NS(name=name, offset=i*4, datatype=7, count=1) for i, name in enumerate(('x', 'y', 'z'))]
        data = np.array([1., 2., .3], dtype='>f4').tobytes() + b'xxxx'
        message = NS(fields=fields, point_step=12, row_step=16, width=1, height=1, data=data, is_bigendian=True)
        np.testing.assert_allclose(cloud_xyz(message), [[1., 2., .3]], atol=1e-7)
        message.data = b'bad'
        with self.assertRaises(ValueError):
            cloud_xyz(message)

    def test_invalid_cloud_is_not_valid_empty_space(self):
        fields = [NS(name=name, offset=i*4, datatype=7, count=1) for i, name in enumerate(('x', 'y', 'z'))]
        message = NS(fields=fields, point_step=12, row_step=12, width=1, height=1,
                     data=np.full(3, np.nan, dtype='<f4').tobytes(), is_bigendian=False)
        with self.assertRaises(ValueError):
            cloud_xyz(message)
        message.width, message.row_step, message.data = 0, 0, b''
        self.assertEqual(cloud_xyz(message).shape, (0, 3))

    def test_cloud_only_marks_obstacles_without_refreshing_depth(self):
        grid = RollingMap(size=21)
        mark_obstacles(grid, np.array([[.5, 0., .3]]), 3.)
        self.assertIsNone(grid.last_depth)
        self.assertFalse(grid.layers(3.)[0].any())
        self.assertTrue(grid.occupied[grid.cell(.5, 0.)])
        mark_obstacles(grid, np.empty((0, 3)), 4.)
        self.assertTrue(grid.occupied[grid.cell(.5, 0.)])

    def test_cloud_transform_matches_target_transform(self):
        tf = NS(rotation=NS(w=math.sqrt(.5), x=0., y=0., z=math.sqrt(.5)), translation=vector(2., 3., 4.))
        point = vector(1., 2., .3)
        np.testing.assert_allclose(transform_points(np.array([[point.x, point.y, point.z]]), tf)[0], transform_point(point, tf))

    def test_real_input_changes_time_and_interfaces_not_optimizer(self):
        interface = hardware_interface('local_odom', 'robot_base', 'real_depth_optical')
        self.assertFalse(interface.use_sim_time)
        self.assertTrue(interface.automatic_follow)
        self.assertEqual(interface.operator_topic, '')
        self.assertFalse(RosInterface().automatic_follow)
        self.assertEqual(interface.command_topic, '/cmd_vel')
        original, hardware = parameters(), parameters(interface)
        key = '/go2_follow_mppi/controller_server'
        old, new = original[key]['ros__parameters'], hardware[key]['ros__parameters']
        self.assertEqual(old['FollowPath'], new['FollowPath'])
        self.assertEqual(old['Observe'], new['Observe'])
        self.assertEqual(old['controller_frequency'], new['controller_frequency'])
        self.assertEqual(new['odom_topic'], interface.odom_topic)
        for entry in hardware.values():
            self.assertFalse(entry['ros__parameters']['use_sim_time'])
        costmap = hardware['/go2_follow_mppi/local_costmap/local_costmap']['ros__parameters']
        self.assertEqual(costmap['global_frame'], 'local_odom')
        self.assertEqual(costmap['robot_base_frame'], 'robot_base')
        shadow = hardware_interface('local_odom', 'robot_base', 'real_depth_optical', '/go2_follow/cmd_vel_candidate')
        self.assertEqual(shadow.command_topic, '/go2_follow/cmd_vel_candidate')
        self.assertEqual(parameters(shadow), hardware)

    def test_rejected_runtime_does_not_delete_another_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'processes.json'
            foreign = [dict(pid=123, start_ticks='4', command='existing owner')]
            path.write_text(json.dumps(foreign))
            with patch('follow_demo.mppi_runtime.MANIFEST', path):
                MppiRuntime().close()
            self.assertEqual(json.loads(path.read_text()), foreign)

    def test_runtime_removes_its_own_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'processes.json'
            runtime = MppiRuntime()
            runtime.records = [dict(pid=123, start_ticks='4', command='owned child')]
            path.write_text(json.dumps(runtime.records))
            with patch('follow_demo.mppi_runtime.MANIFEST', path):
                runtime.close()
            self.assertFalse(path.exists())


if __name__ == '__main__':
    unittest.main()
