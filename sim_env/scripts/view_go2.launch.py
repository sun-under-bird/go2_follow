"""独立展示 Go2 URDF，不启动任何底盘命令节点。"""
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """加载已安装模型，并启动关节调节界面与 RViz。"""
    description = Path(get_package_share_directory('go2_description')) / 'urdf/go2_description.urdf'
    urdf = description.read_text()
    zeros = {}
    for joint in ET.fromstring(urdf).findall('joint'):
        name = joint.attrib['name']
        if joint.attrib['type'] != 'fixed':
            zeros['zeros.' + name] = -1.5 if 'calf' in name.lower() else (0.8 if 'thigh' in name.lower() else 0.0)
    return LaunchDescription([
        # 发布模型和各连杆 TF，仅用于模型查看。
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}], output='screen'),
        # 用滑条调整 12 个关节，检查 URDF 关节与网格是否正常。
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[zeros], output='screen'),
        # 以 base_link 为固定坐标系显示机器人，不依赖里程计。
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(Path.home() / 'go2_sim/config/go2.rviz')], output='screen'),
    ])
