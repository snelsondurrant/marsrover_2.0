# Created by Nelson Durrant, July 2025
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    coug_des_dir = get_package_share_directory("coug_description")
    coug_des_launch_dir = os.path.join(coug_des_dir, "launch")
    coug_loc_dir = get_package_share_directory("coug_localization")
    coug_loc_launch_dir = os.path.join(coug_loc_dir, "launch")
    rover_gui_dir = get_package_share_directory("rover_gui")
    rover_gui_launch_dir = os.path.join(rover_gui_dir, "launch")

    coug_des_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_des_launch_dir, "coug_description.launch.py")
        ),
    )

    coug_loc_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coug_loc_launch_dir, "coug_localization.launch.py")
        ),
    )

    rover_mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_gui_launch_dir, "mapviz.launch.py")
        ),
    )

    rover_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_gui_launch_dir, "rviz.launch.py")
        ),
    )

    ld = LaunchDescription()
    ld.add_action(coug_des_cmd)
    ld.add_action(coug_loc_cmd)
    ld.add_action(rover_mapviz_cmd)
    ld.add_action(rover_rviz_cmd)

    return ld
