"""UWB random target generation without a chassis velocity publisher."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = Path(get_package_share_directory("go2_uwb_behavior"))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "behavior_follow_roam.launch.py")),
            launch_arguments={"target_only": "true", "enable_motion": "false"}.items(),
        ),
    ])
