# Created by Nelson Durrant, Mar 2025
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    description_dir = get_package_share_directory(
        "rover_description")
    urdf = os.path.join(description_dir, 'urdf', 'turtlebot3_waffle_gps.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # robot state publisher launch
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
