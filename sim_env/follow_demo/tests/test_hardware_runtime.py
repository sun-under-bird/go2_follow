"""真实消息乱序、TF 延迟及自动运行时的实际传感器边界。"""
from types import MethodType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from tf2_ros import TransformException

from follow_demo.hardware_runtime import HardwareInputs
from follow_demo.controller import Pose
from follow_demo.local_planner import Plan
from follow_demo.navigation import NavigationController
from follow_demo.ros_nodes import FollowerNode
from follow_demo.ros_contract import hardware_interface


def relay():
    node = NS(depths={}, clouds={}, pending_clouds={}, last_pair=-1,
              cloud_pub=Mock(), info_pub=Mock(), depth_pub=Mock(), warn=Mock(),
              interface=hardware_interface('odom', 'base_footprint', 'real_optical'))
    node.stamp_key = HardwareInputs.stamp_key
    for name in ('publish_pair', 'retry_clouds'):
        setattr(node, name, MethodType(getattr(HardwareInputs, name), node))
    return node


class HardwareRuntimeTests(unittest.TestCase):
    def test_different_capture_times_cannot_refresh_depth(self):
        node = relay()
        node.depths[100] = ('depth100', 'info100')
        node.clouds[101] = 'cloud101'
        node.publish_pair(101)
        node.depth_pub.publish.assert_not_called()
        node.clouds[100] = 'cloud100'
        node.publish_pair(100)
        node.depth_pub.publish.assert_called_once_with('depth100')
        node.cloud_pub.publish.assert_called_once_with('cloud100')
        node.depths[99], node.clouds[99] = ('old_depth', 'old_info'), 'old_cloud'
        node.publish_pair(99)
        node.depth_pub.publish.assert_called_once_with('depth100')

    def test_delayed_tf_retries_original_capture_time(self):
        node = relay()
        transform = NS(rotation=NS(w=1., x=0., y=0., z=0.), translation=NS(x=1., y=0., z=0.))
        node.tf_buffer = NS(lookup_transform=Mock(side_effect=[TransformException('not arrived'), NS(transform=transform)]))
        cloud = PointCloud2(header=Header(stamp=Time(sec=2, nanosec=123), frame_id='base_footprint'),
                            width=1, height=1, point_step=12, row_step=12,
                            fields=[PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                                    for i, name in enumerate(('x', 'y', 'z'))],
                            data=np.array([1., 0., .3], dtype='<f4').tobytes())
        key = node.stamp_key(cloud)
        node.depths[key] = ('same_depth', 'same_info')
        HardwareInputs.on_cloud(node, cloud)
        node.depth_pub.publish.assert_not_called()
        node.retry_clouds()
        output = node.cloud_pub.publish.call_args.args[0]
        self.assertEqual(output.header.stamp, cloud.header.stamp)
        self.assertEqual(output.header.frame_id, 'odom')
        np.testing.assert_allclose(np.frombuffer(output.data, dtype='<f4'), [2., 0., .3])
        node.depth_pub.publish.assert_called_once_with('same_depth')
        stamps = [call.args[2].nanoseconds for call in node.tf_buffer.lookup_transform.call_args_list]
        self.assertEqual(stamps, [key, key])
        self.assertFalse(node.pending_clouds)

    def automatic_node(self, require_start_confirmation=False):
        core = NavigationController(require_start_confirmation=require_start_confirmation)
        self.addCleanup(core.close)
        core.history.add(Pose(1., 0., 0., 0.))
        core.observe(4., 0., 1.)
        core.grid.seen[:] = 1.
        core.grid.last_depth = 1.
        core.plan = Plan('FOLLOWING', 'FOLLOW_REGION_REACHABLE', [[0., 0.], [2., 0.]])
        core.last_plan = 1.
        core.external_command, core.external_stamp, core.external_active = (.3, 0.), 1., True
        # 这些旧操作字段即便为暂停/急停/心跳过期，也不能影响自动入口。
        return NS(core=core, interface=hardware_interface('odom', 'base_footprint', 'real_optical'),
                  operator=dict(enabled=False, emergency=True, ready=False, signal=False), last_operator_wall=0.)

    def test_automatic_follow_moves_without_operator_or_clearance_confirmation(self):
        node = self.automatic_node()
        command = FollowerNode.controller_command(node, 1.)
        self.assertGreater(command[0], 0.)
        self.assertFalse(node.core.grid.confirmed)
        self.assertNotEqual(node.core.code, 'START_UNCONFIRMED')
        self.assertNotEqual(node.core.code, 'OPERATOR_STALE')

    def test_simulation_still_requires_start_confirmation(self):
        node = self.automatic_node(require_start_confirmation=True)
        self.assertEqual(node.core.step(1.), (0., 0.))
        self.assertEqual(node.core.code, 'START_UNCONFIRMED')

    def test_automatic_start_cannot_invent_free_space(self):
        node = self.automatic_node()
        node.core.grid.seen[:] = -np.inf
        self.assertEqual(FollowerNode.controller_command(node, 1.), (0., 0.))
        self.assertEqual(node.core.code, 'FOOTPRINT_UNKNOWN')
        self.assertFalse(np.isfinite(node.core.grid.seen).any())
        self.assertFalse(node.core.grid.confirmed)

    def test_automatic_follow_stops_for_each_stale_sensor(self):
        for stale, code in (('target', 'TARGET_INVALID'), ('odom', 'ODOM_STALE'), ('depth', 'MAP_STALE')):
            with self.subTest(stale=stale):
                node = self.automatic_node()
                if stale == 'target':
                    node.core.target_stamp = .4
                elif stale == 'odom':
                    node.core.history.values.clear()
                else:
                    node.core.grid.last_depth = 0.
                self.assertEqual(FollowerNode.controller_command(node, 1.), (0., 0.))
                self.assertEqual(node.core.code, code)


if __name__ == '__main__':
    unittest.main()
