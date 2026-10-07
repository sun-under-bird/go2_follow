"""D435i 红外双目的简化几何配置；安装位置和实际设备标定仍待实测。"""
import math

# 采用 D435i 所用 D430 深度模组的约 50 mm 基线与约 58° 垂直视场。
# 分辨率选 424×240，理想针孔的水平视场约 88.8°；不是某台设备导出的内参。
WIDTH, HEIGHT = 424, 240
FOVY, BASELINE = 58.0, 0.050
HFOV = math.degrees(2 * math.atan(WIDTH / HEIGHT * math.tan(math.radians(FOVY / 2))))

# 原点为双目中点，机身坐标为前 x、左 y、上 z；此安装位置是待测占位值。
CAMERA_CENTER = (0.23, 0.025, 0.135)
# 保留水平安装作为默认对照；试验俯角必须显式传入，不能冒充实机外参。
DEFAULT_CAMERA_PITCH_DEG = 0.0


def mounting_geometry(pitch_deg=DEFAULT_CAMERA_PITCH_DEG):
    """返回向下俯角对应的 MuJoCo 相机轴与 ROS 光学/IMU 四元数（xyzw）。"""
    if not math.isfinite(pitch_deg) or not 0 <= pitch_deg <= 40:
        raise ValueError('仿真相机俯角必须是 0～40 度的有限值')
    pitch = math.radians(pitch_deg)
    sine, cosine = math.sin(pitch / 2), math.cos(pitch / 2)
    # MuJoCo 相机的右、上轴；ROS 光学坐标则是右、下、前，不能直接复用。
    axes = (0., -1., 0., math.sin(pitch), 0., math.cos(pitch))
    optical = (-.5 * (cosine + sine), .5 * (cosine + sine),
               -.5 * (cosine - sine), .5 * (cosine - sine))
    imu = (0., sine, 0., cosine)
    return axes, optical, imu


def camera_position(side):
    """按双目中点计算左右光心位置，让 MJCF 和 ROS TF 使用同一份外参。"""
    sign = {'left': 1, 'right': -1}[side]
    x, y, z = CAMERA_CENTER
    return [x, y + sign * BASELINE / 2, z]


def camera_intrinsic(width, height):
    """按MuJoCo像素中心生成理想内参；整数坐标对应像素中心，主点需减半个像素。"""
    focal = height / (2 * math.tan(math.radians(FOVY / 2)))
    return focal, focal, (width - 1) / 2, (height - 1) / 2
