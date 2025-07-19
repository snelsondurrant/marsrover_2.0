# Created by Nelson Durrant, July 2025
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rover_gtsam")
    gtsam_params_file = os.path.join(pkg_share, "config", "gtsam_params.yaml")

    gtsam_localizer_node = Node(
        package="rover_gtsam",
        executable="gtsam_localizer_node",
        name="gtsam_localizer_node",
        output="screen",
        parameters=[gtsam_params_file, {"use_sim_time": True}],
    )

    return LaunchDescription([gtsam_localizer_node])
