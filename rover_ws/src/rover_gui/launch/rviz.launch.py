# Created by Nelson Durrant, Mar 2025
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

bringup_dir = get_package_share_directory("nav2_bringup")

use_sim_time = LaunchConfiguration("use_sim_time")
declare_use_sim_time_cmd = DeclareLaunchArgument(
    "use_sim_time", default_value="False", description="Use simulation time"
)

# IMPORTANT! We've found that running rviz2 on the base station computer and sending Nav2 goals
# through it over the network to the rover results in a lot of timing errors. It works much better
# when those goals are sent from the rover computer itself (e.g. in the state machine).


def generate_launch_description():
    return launch.LaunchDescription(
        [
            declare_use_sim_time_cmd,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, "launch", "rviz_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ]
    )
