#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class StereoSplitNode(Node):
    """把左右拼接双目图从中线拆成左右两路，并保持同一时间戳。"""

    def __init__(self):
        """初始化参数、发布订阅和标定信息。"""
        super().__init__("stereo_split_node")

        self.declare_parameter("input_topic", "/image_raw")
        self.declare_parameter("left_image_topic", "/stereo/left/camera/image_mono")
        self.declare_parameter("right_image_topic", "/stereo/right/camera/image_mono")
        self.declare_parameter("left_info_topic", "/stereo/left/camera/camera_info")
        self.declare_parameter("right_info_topic", "/stereo/right/camera/camera_info")
        self.declare_parameter("left_info_url", "")
        self.declare_parameter("right_info_url", "")
        self.declare_parameter("left_frame", "left_camera_optical_frame")
        self.declare_parameter("right_frame", "right_camera_optical_frame")
        self.declare_parameter("output_encoding", "mono8")
        self.declare_parameter("max_publish_rate_hz", 40.0)

        self.left_frame = str(self.get_parameter("left_frame").value)
        self.right_frame = str(self.get_parameter("right_frame").value)
        self.output_encoding = str(self.get_parameter("output_encoding").value)
        self.max_publish_rate_hz = float(self.get_parameter("max_publish_rate_hz").value)
        self.last_publish_time = None

        self.bridge = CvBridge()
        self.left_info = self._load_camera_info(
            str(self.get_parameter("left_info_url").value),
            self.left_frame,
        )
        self.right_info = self._load_camera_info(
            str(self.get_parameter("right_info_url").value),
            self.right_frame,
        )

        self.left_pub = self.create_publisher(
            Image,
            str(self.get_parameter("left_image_topic").value),
            qos_profile_sensor_data,
        )
        self.right_pub = self.create_publisher(
            Image,
            str(self.get_parameter("right_image_topic").value),
            qos_profile_sensor_data,
        )
        self.left_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("left_info_topic").value),
            qos_profile_sensor_data,
        )
        self.right_info_pub = self.create_publisher(
            CameraInfo,
            str(self.get_parameter("right_info_topic").value),
            qos_profile_sensor_data,
        )
        self.sub = self.create_subscription(
            Image,
            str(self.get_parameter("input_topic").value),
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().warn("stereo_split_node started")

    def _load_camera_info(self, url: str, frame_id: str) -> CameraInfo:
        """从 ROS camera_calibration YAML 中读取 CameraInfo。"""
        info = CameraInfo()
        info.header.frame_id = frame_id
        path = self._normalize_url(url)
        if path is None:
            return info

        try:
            with open(path, "r", encoding="utf-8") as file:
                calib = yaml.safe_load(file) or {}
            info.width = int(calib.get("image_width", 0))
            info.height = int(calib.get("image_height", 0))
            info.k = list(calib.get("camera_matrix", {}).get("data", [0.0] * 9))
            info.d = list(calib.get("distortion_coefficients", {}).get("data", []))
            info.r = list(calib.get("rectification_matrix", {}).get("data", [0.0] * 9))
            info.p = list(calib.get("projection_matrix", {}).get("data", [0.0] * 12))
            info.distortion_model = str(calib.get("distortion_model", "plumb_bob"))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"读取相机标定失败 {path}: {exc}")
        return info

    def _normalize_url(self, url: str) -> Optional[str]:
        """把 file:// 或普通路径统一成可打开的本地路径。"""
        value = url.strip()
        if not value:
            return None
        if value.startswith("file://"):
            return value[len("file://") :]
        return value

    def _should_publish(self) -> bool:
        """根据 max_publish_rate_hz 对高帧率相机做轻量限频。"""
        if self.max_publish_rate_hz <= 0.0 or not math.isfinite(self.max_publish_rate_hz):
            return True
        now = self.get_clock().now()
        if self.last_publish_time is None:
            self.last_publish_time = now
            return True
        elapsed = (now - self.last_publish_time).nanoseconds * 1e-9
        if elapsed < 1.0 / self.max_publish_rate_hz:
            return False
        self.last_publish_time = now
        return True

    def _image_callback(self, msg: Image) -> None:
        """接收拼接图，按中线切成左右图并发布。"""
        if not self._should_publish():
            return

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.output_encoding)
        width = image.shape[1]
        half_width = width // 2
        if half_width <= 0:
            self.get_logger().warn("拼接图宽度无效，跳过本帧")
            return

        # 左右图来自同一帧拼接图，必须共用原始时间戳，保证 stereo 同步。
        left_image = image[:, :half_width]
        right_image = image[:, half_width:]
        left_msg = self.bridge.cv2_to_imgmsg(left_image, encoding=self.output_encoding)
        right_msg = self.bridge.cv2_to_imgmsg(right_image, encoding=self.output_encoding)
        left_msg.header.stamp = msg.header.stamp
        right_msg.header.stamp = msg.header.stamp
        left_msg.header.frame_id = self.left_frame
        right_msg.header.frame_id = self.right_frame

        self._prepare_info(self.left_info, msg.header.stamp, self.left_frame, half_width, image.shape[0])
        self._prepare_info(
            self.right_info,
            msg.header.stamp,
            self.right_frame,
            width - half_width,
            image.shape[0],
        )

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        self.left_info_pub.publish(self.left_info)
        self.right_info_pub.publish(self.right_info)

    def _prepare_info(
        self,
        info: CameraInfo,
        stamp,
        frame_id: str,
        width: int,
        height: int,
    ) -> None:
        """补齐 CameraInfo 的时间戳、frame 和缺省尺寸。"""
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        if info.width == 0:
            info.width = width
        if info.height == 0:
            info.height = height


def main(args=None):
    """启动 stereo_split_node 节点。"""
    rclpy.init(args=args)
    node = StereoSplitNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
