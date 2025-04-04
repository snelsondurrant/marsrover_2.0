# Created by Nelson Durrant, Feb 2025
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
import launch_ros.actions
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation time"
    )

    nav_dir = get_package_share_directory("rover_navigation")
    config_file = os.path.join(nav_dir, "config", "navigation_params.yaml")
    sim_config_file = os.path.join(nav_dir, "config", "sim_navigation_params.yaml")

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            launch_ros.actions.Node(
                # Easier to include this in the sim than refactor services
                package="rover_control",
                executable="drive_mux",
                output="screen",
                condition=IfCondition(use_sim_time),
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            launch_ros.actions.Node(
                package="rover_navigation",
                executable="autonomy_task_executor",
                output="screen",
                parameters=[
                    config_file,
                    {"use_sim_time": use_sim_time},
                ],
                condition=UnlessCondition(use_sim_time),
            ),
            launch_ros.actions.Node(
                package="rover_navigation",
                executable="autonomy_task_executor",
                output="screen",
                parameters=[
                    sim_config_file,
                    {"use_sim_time": use_sim_time},
                ],
                condition=IfCondition(use_sim_time),
            ),
        ]
    )
