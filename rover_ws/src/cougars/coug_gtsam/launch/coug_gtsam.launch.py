# Created by Nelson Durrant, July 2025
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("coug_gtsam")
    gtsam_dvl_params_file = os.path.join(pkg_share, "config", "gtsam_dvl_params.yaml")

    gtsam_dvl_localizer_node = Node(
        package="coug_gtsam",
        executable="gtsam_dvl_localizer_node",
        name="gtsam_dvl_localizer_node",
        output="screen",
        parameters=[gtsam_dvl_params_file],
    )

    return LaunchDescription([gtsam_dvl_localizer_node])
