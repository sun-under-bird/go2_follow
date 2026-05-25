from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# 创建启动描述，声明参数文件并启动 UWB 路径跟随节点。
def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("go2_uwb_mppi_follow"),
            "config",
            "uwb_mppi_follow.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="UWB MPPI follow parameter file.",
            ),
            Node(
                package="go2_uwb_mppi_follow",
                executable="uwb_path_tracker_node",
                name="uwb_path_tracker_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
