# Modified from the GPS demo - Nelson Durrant, Feb 2025
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

nav_dir = get_package_share_directory("rover_navigation")
sim_rviz_config_file = os.path.join(nav_dir, "config", "sim_rviz_params.rviz")
rviz_config_file = os.path.join(nav_dir, "config", "rviz_params.rviz")

use_sim_time = LaunchConfiguration('use_sim_time')
declare_use_sim_time_cmd = DeclareLaunchArgument(
    'use_sim_time',
    default_value='False',
    description='Use simulation time')


def generate_launch_description():
    return launch.LaunchDescription([
        declare_use_sim_time_cmd,
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            parameters=[{"use_sim_time": use_sim_time, "config": rviz_config_file}],
            condition=UnlessCondition(use_sim_time),
        ),
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            parameters=[{"use_sim_time": use_sim_time, "config": sim_rviz_config_file}],
            condition=IfCondition(use_sim_time),
        ),
    ])
