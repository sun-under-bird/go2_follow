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

"""Start the UWB driver, follow/roam controller, and visual target localization."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
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
            description="Stable UWB device path for this robot; override for another adapter.",
        ),
        DeclareLaunchArgument(
            "enable_motion",
            default_value="true",
            description="Enable UWB follow/roam motion, as in the original command.",
        ),
        DeclareLaunchArgument(
            "person_localization_config",
            default_value=str(
                localization_share / "config" / "person_3d_localization.yaml"
            ),
            description="Configuration file for visual target depth localization.",
        ),
        # Scope each include so its defaults cannot change another launch's
        # parameters. Reuse the original launches without creating extra nodes.
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                uwb_share / "launch" / "uwb_source.launch.py"
            )),
            launch_arguments={
                "serial_port": LaunchConfiguration("serial_port"),
            }.items(),
        )]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                behavior_share / "launch" / "behavior_follow_roam.launch.py"
            )),
            launch_arguments={
                "enable_motion": LaunchConfiguration("enable_motion"),
            }.items(),
        )]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                localization_share / "launch" / "person_3d_localization.launch.py"
            )),
            launch_arguments={
                "config_file": LaunchConfiguration("person_localization_config"),
            }.items(),
        )]),
    ])
