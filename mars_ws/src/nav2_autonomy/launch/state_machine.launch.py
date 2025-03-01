# Created by Nelson Durrant, Feb 2025
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():
    gps_wpf_dir = get_package_share_directory(
        "nav2_autonomy")
    rl_params_file = os.path.join(
        gps_wpf_dir, "config", "state_machine_params.yaml")
    wps_file = os.path.join(
        gps_wpf_dir, "waypoints/output", "sim_basic_waypoints.yaml")

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package = "nav2_autonomy",
                executable = "state_machine",
                output = "screen",
                parameters = [ rl_params_file, {"use_sim_time": True, "wps_file_path": wps_file}],
            ),
        ]
    )
