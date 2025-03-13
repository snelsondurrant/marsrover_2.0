# Created by Nelson Durrant, Mar 2025
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation time"
    )

    # Get the launch directory
    description_dir = get_package_share_directory(
        "rover_description")
    if use_sim_time:
        urdf = os.path.join(description_dir, 'urdf', 'rover.urdf')
    else:
        urdf = os.path.join(description_dir, 'urdf', 'rover.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # robot state publisher launch
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
