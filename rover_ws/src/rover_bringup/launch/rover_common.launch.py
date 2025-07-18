# Created by Nelson Durrant, Feb 2025
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    control_dir = get_package_share_directory("rover_control")
    control_launch_dir = os.path.join(control_dir, "launch")

    control_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_launch_dir, "rover_control.launch.py")
        ),
    )

    ld = LaunchDescription()
    ld.add_action(control_cmd)

    return ld
