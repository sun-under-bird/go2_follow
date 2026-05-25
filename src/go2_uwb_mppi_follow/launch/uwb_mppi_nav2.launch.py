from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# 创建完整启动描述，同时启动 UWB 路径节点和 Nav2 MPPI 控制器。
def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    uwb_params_file = LaunchConfiguration("uwb_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    default_uwb_params_file = PathJoinSubstitution(
        [
            FindPackageShare("go2_uwb_mppi_follow"),
            "config",
            "uwb_mppi_follow.yaml",
        ]
    )
    default_nav2_params_file = PathJoinSubstitution(
        [
            FindPackageShare("go2_uwb_mppi_follow"),
            "config",
            "nav2_mppi_controller.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically configure and activate Nav2 lifecycle nodes.",
            ),
            DeclareLaunchArgument(
                "uwb_params_file",
                default_value=default_uwb_params_file,
                description="Parameter file for the UWB path tracker node.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params_file,
                description="Parameter file for Nav2 controller server and local costmap.",
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_controller",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": ["controller_server"]},
                ],
            ),
            Node(
                package="go2_uwb_mppi_follow",
                executable="uwb_path_tracker_node",
                name="uwb_path_tracker_node",
                output="screen",
                parameters=[uwb_params_file, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
