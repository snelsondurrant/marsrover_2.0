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

use_sim_time = LaunchConfiguration('use_sim_time')
declare_use_sim_time_cmd = DeclareLaunchArgument(
    'use_sim_time',
    default_value='False',
    description='Use simulation time')


def generate_launch_description():
    return launch.LaunchDescription([
        declare_use_sim_time_cmd,
        launch_ros.actions.Node(
            package="rqt_console",
            executable="rqt_console",
            name="rqt_console",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])