# Created by Nelson Durrant, Feb 2025
import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

gui_dir = get_package_share_directory("rover_gui")
mapviz_config_file = os.path.join(gui_dir, "config", "mapviz_params.mvc")

use_sim_time = LaunchConfiguration("use_sim_time")
declare_use_sim_time_cmd = DeclareLaunchArgument(
    "use_sim_time", default_value="False", description="Use simulation time"
)


def generate_launch_description():
    return launch.LaunchDescription(
        [
            declare_use_sim_time_cmd,
            launch_ros.actions.Node(
                package="mapviz",
                executable="mapviz",
                name="mapviz",
                parameters=[
                    {"config": mapviz_config_file, "use_sim_time": use_sim_time}
                ],
            ),
            launch_ros.actions.Node(
                package="swri_transform_util",
                executable="initialize_origin.py",
                name="initialize_origin",
                remappings=[
                    ("fix", "mapviz/origin"),  # set by the sync_origin node
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="swri_transform",
                arguments=["0", "0", "0", "0", "0", "0", "map", "origin"],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
