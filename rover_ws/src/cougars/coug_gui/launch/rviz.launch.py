# Created by Nelson Durrant, July 2025
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("coug_gui")
    rviz_params_file = os.path.join(pkg_share, "config", "rviz_params.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_params_file],
    )

    return LaunchDescription([rviz_node])
