# Created by Nelson Durrant, Feb 2025
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation time"
    )

    sim_wps_file = (
        "/home/marsrover-docker/rover_ws/src/rover_navigation/config/sim_waypoints.yaml"
    )
    wps_file = (
        "/home/marsrover-docker/rover_ws/src/rover_navigation/config/waypoints.yaml"
    )

    nav_dir = get_package_share_directory("rover_navigation")
    config_file = os.path.join(nav_dir, "config", "nav2_no_map_params.yaml")
    sim_config_file = os.path.join(nav_dir, "config", "sim_nav2_no_map_params.yaml")

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
                    {"use_sim_time": use_sim_time, "wps_file_path": wps_file},
                ],
                condition=UnlessCondition(use_sim_time),
            ),
            launch_ros.actions.Node(
                package="rover_navigation",
                executable="autonomy_task_executor",
                output="screen",
                parameters=[
                    sim_config_file,
                    {"use_sim_time": use_sim_time, "wps_file_path": sim_wps_file},
                ],
                condition=IfCondition(use_sim_time),
            ),
        ]
    )
