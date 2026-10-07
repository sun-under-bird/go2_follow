"""同源消息异步接入反例：未来一点的里程计不应让有效旧样本被误判过期。"""
import unittest
from follow_demo.controller import Pose,PoseHistory
from follow_demo.navigation import NavigationController


class OdometryAlignmentTests(unittest.TestCase):
    """保留测量时间和同帧速度，控制时刻不改写任何传感器时间。"""
    def test_future_delivery_waits_without_invalidating_fresh_past_sample(self):
        """复现旧逻辑负年龄误停车；同帧位姿/速度延后到时钟覆盖才生效。"""
        history=PoseHistory()
        past=Pose(1.,0.,0.,0.,(.2,0.,.1))
        future=Pose(1.04,.1,0.,.2,(.8,0.,1.))
        history.add(past)
        history.add(future)
        self.assertLess(1.02-history.values[-1].t,0.)
        self.assertIs(history.current(1.02),past)
        self.assertEqual(history.current(1.02).motion,(.2,0.,.1))
        self.assertIs(history.current(1.04),future)
        self.assertEqual(future.t,1.04)

    def test_only_future_or_truly_stale_sample_cannot_pass_health_gate(self):
        """仍拒收无过去覆盖及真正超时的测量，不以新到包或未来包刷新健康。"""
        history=PoseHistory()
        history.add(Pose(.7,0.,0.,0.))
        history.add(Pose(1.04,0.,0.,0.))
        self.assertIsNone(history.current(1.02))
        self.assertIsNone(history.current(.5))
        self.assertIsNone(history.current(float('nan')))

    def test_navigation_uses_time_aligned_pose_and_motion(self):
        """控制实际使用过去样本，而不是仅修改诊断或放宽负年龄阈值。"""
        core=NavigationController()
        self.addCleanup(core.close)
        core.history.add(Pose(1.,0.,0.,0.,(.2,0.,.1)))
        core.observe(6.,0.,1.)
        core.history.add(Pose(1.04,.1,0.,.2,(.8,0.,1.)))
        core.grid.seen[:]=1.
        core.grid.last_depth,core.grid.confirmed=1.,True
        core.step(1.02)
        self.assertNotEqual(core.code,'ODOM_STALE')
        self.assertEqual(core.control_pose.t,1.)
        self.assertEqual(core.motion,(.2,0.,.1))
        core.step(1.3)
        self.assertEqual(core.code,'ODOM_STALE')

    def test_start_confirmation_uses_same_timestamp_rule(self):
        """起始确认也不被正常消息交错拒绝，真正无覆盖时仍拒绝确认。"""
        core=NavigationController()
        self.addCleanup(core.close)
        core.history.add(Pose(1.,0.,0.,0.))
        core.history.add(Pose(1.04,.1,0.,0.))
        self.assertTrue(core.confirm_start(1.02))


if __name__=='__main__':
    unittest.main()
