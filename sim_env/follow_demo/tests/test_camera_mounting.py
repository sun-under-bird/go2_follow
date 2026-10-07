"""核对安装角度的渲染、传感器坐标系与验收身份，防止只旋转图像却遗漏 TF。"""
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

import mujoco
import numpy as np

from follow_demo.camera_profile import mounting_geometry
from follow_demo.simulation import make_scene
from follow_demo.ros_nodes import SensorBridge, timestamp
from follow_demo.tests.check_navigation_demo import verify_camera_pitch


def rotation_xyzw(quaternion):
    """把 ROS 顺序四元数转换为旋转矩阵，独立核对不同坐标约定。"""
    x, y, z, w = quaternion
    matrix = np.empty(9)
    mujoco.mju_quat2Mat(matrix, np.array([w, x, y, z], dtype=float))
    return matrix.reshape(3, 3)


class CameraMountingTests(unittest.TestCase):
    """几何检查不依赖闭环控制是否偶然通过。"""
    def test_scene_tf_and_imu_share_downward_rotation(self):
        """四种角度下，MJCF 相机轴、ROS 光学轴和 IMU 轴必须对应同一安装。"""
        for pitch in (0., 15., 25., 40.):
            with self.subTest(pitch=pitch), TemporaryDirectory() as temporary:
                axes, optical, imu = mounting_geometry(pitch)
                model = mujoco.MjModel.from_xml_path(str(make_scene(Path(temporary), pitch)))
                theta = math.radians(pitch)
                expected_imu = np.array([[math.cos(theta), 0., math.sin(theta)],
                                         [0., 1., 0.], [-math.sin(theta), 0., math.cos(theta)]])
                np.testing.assert_allclose(rotation_xyzw(imu), expected_imu, atol=1e-12)
                camera_matrix = rotation_xyzw(optical) @ np.diag([1., -1., -1.])
                for side in ('left', 'right'):
                    quaternion = model.cam_quat[model.camera(f'front_{side}').id]
                    np.testing.assert_allclose(rotation_xyzw((*quaternion[1:], quaternion[0])), camera_matrix, atol=1e-12)
                np.testing.assert_allclose(camera_matrix[:, :2].T.ravel(), axes, atol=1e-12)
                site_q = model.site_quat[model.site('camera_imu').id]
                np.testing.assert_allclose(rotation_xyzw((*site_q[1:], site_q[0])), expected_imu, atol=1e-12)
                published = []
                bridge = SimpleNamespace(simulation=SimpleNamespace(camera_pitch_deg=pitch),
                                         tf=SimpleNamespace(sendTransform=published.extend))
                # 直接调用消息生成函数，避免为了几何检查启动 ROS 节点或物理服务。
                SensorBridge.publish_tf(bridge, dict(x=0., y=0., z=.32, yaw=0., quaternion=[1., 0., 0., 0.]), timestamp(1.))
                for transform in published[2:]:
                    q = transform.transform.rotation
                    expected = expected_imu if transform.child_frame_id == 'camera_imu' else rotation_xyzw(optical)
                    np.testing.assert_allclose(rotation_xyzw((q.x, q.y, q.z, q.w)), expected, atol=1e-12)

    def test_invalid_mounts_are_rejected(self):
        """缺少标定或非有限参数不能悄悄变成另一种安装。"""
        for value in (-1., 40.01, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                mounting_geometry(value)

    def test_acceptance_rejects_missing_mixed_or_wrong_pitch(self):
        """不同角度的采样必须被验收脚本识别，避免错误的对照结论。"""
        self.assertEqual(verify_camera_pitch([dict(camera_pitch_deg=25.)], 25.), 25.)
        for rows, expected in (([], None), ([{}], None), ([dict(camera_pitch_deg=0.)], 25.),
                               ([dict(camera_pitch_deg=0.), dict(camera_pitch_deg=25.)], None),
                               ([dict(camera_pitch_deg=float('nan'))], None)):
            with self.assertRaises(RuntimeError):
                verify_camera_pitch(rows, expected)


if __name__ == '__main__':
    unittest.main()
