# Created by Nelson Durrant, Mar 2025
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

bringup_dir = get_package_share_directory('nav2_bringup')
nav_dir = get_package_share_directory("rover_navigation")
# rviz_config_file = os.path.join(nav_dir, "config", "rviz_params.rviz")

use_sim_time = LaunchConfiguration('use_sim_time')
declare_use_sim_time_cmd = DeclareLaunchArgument(
    'use_sim_time',
    default_value='False',
    description='Use simulation time')


def generate_launch_description():
    return launch.LaunchDescription([
        declare_use_sim_time_cmd,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "launch", 'rviz_launch.py')),
            # condition=IfCondition(use_sim_time),
            launch_arguments={
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        # launch_ros.actions.Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     parameters=[{"use_sim_time": use_sim_time}],
        #     arguments=["-d", rviz_config_file],
        #     condition=UnlessCondition(use_sim_time),
        # ),
    ])
