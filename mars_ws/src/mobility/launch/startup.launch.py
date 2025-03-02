# Created by Nelson Durrant, Feb 2025
import launch
import launch_ros.actions
import launch_ros.descriptions
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mobility_dir = get_package_share_directory(
        "mobility")
    mobility_launch_dir = os.path.join(mobility_dir, 'launch')
    peripherals_dir = get_package_share_directory(
        "peripherals")
    peripherals_launch_dir = os.path.join(peripherals_dir, 'launch')

    mobility_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mobility_launch_dir, 'mobility.launch.py')),
    )

    peripherals_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_launch_dir, 'peripherals.launch.py')),
    )

    ld = LaunchDescription()
    ld.add_action(mobility_cmd)
    ld.add_action(peripherals_cmd)

    return ld
