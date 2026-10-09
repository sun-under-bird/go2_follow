# Copyright 2026 OpenAI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""启动 UWB 驱动、统一运动行为和视觉目标定位."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """复用现有唯一规划链路，并转发多目标及模式隔离参数."""
    uwb_share = Path(get_package_share_directory("uwb_aoa_pkg"))
    behavior_share = Path(get_package_share_directory("go2_uwb_behavior"))
    localization_share = Path(get_package_share_directory("person_3d_localization"))

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value=(
                "/dev/serial/by-id/"
                "usb-FTDI_FT232R_USB_UART_AP2315SD-if00-port0"
            ),
            description="机器人 UWB 串口的稳定路径；其他适配器可覆盖。",
        ),
        DeclareLaunchArgument(
            "enable_motion",
            default_value="true",
            description="是否启用 UWB 运动；false 用于无运动联调。",
        ),
        DeclareLaunchArgument("targets_topic", default_value="/uwb/targets"),
        DeclareLaunchArgument("target_only", default_value="false"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument(
            "person_localization_config",
            default_value=str(
                localization_share / "config" / "person_3d_localization.yaml"
            ),
            description="视觉目标深度定位配置。",
        ),
        # 各包含文件独立作用域，避免同名默认参数相互覆盖。
        # 只启动一个原有 UWB 串口驱动。
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                uwb_share / "launch" / "uwb_source.launch.py"
            )),
            launch_arguments={
                "serial_port": LaunchConfiguration("serial_port"),
            }.items(),
        )]),
        # 统一行为链路只包含一套 MPPI 和一个最终速度出口。
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                behavior_share / "launch" / "behavior_follow_roam.launch.py"
            )),
            launch_arguments={
                "enable_motion": LaunchConfiguration("enable_motion"),
                "target_only": LaunchConfiguration("target_only"),
                "targets_topic": LaunchConfiguration("targets_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            }.items(),
        )]),
        # 原有视觉人员定位独立运行，不参与飞盘拾取或目标自动切换。
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                localization_share / "launch" / "person_3d_localization.launch.py"
            )),
            launch_arguments={
                "config_file": LaunchConfiguration("person_localization_config"),
            }.items(),
        )]),
    ])
