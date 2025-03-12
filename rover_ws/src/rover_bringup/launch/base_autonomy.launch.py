# Created by Nelson Durrant, Mar 2025
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    nav_dir = get_package_share_directory('rover_navigation')
    nav_launch_dir = os.path.join(nav_dir, 'launch')
    ublox_dir = get_package_share_directory('ublox_read_2')
    ublox_launch_dir = os.path.join(ublox_dir, 'launch')

    use_rviz = LaunchConfiguration('use_rviz')
    use_mapviz = LaunchConfiguration('use_mapviz')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        'use_mapviz',
        default_value='False',
        description='Whether to start mapviz')

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, 'mapviz.launch.py')),
        condition=IfCondition(use_mapviz),
    )

    gps_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ublox_launch_dir, 'base_launch.xml')),
    )

    ld = LaunchDescription(
        [
            Node(
                # https://docs.ros.org/en/iron/p/joy/
                package="joy",
                executable="joy_node",
                name="joy_node_base",
                output="screen",
            ),
        ]
    )

    # viz launch
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(mapviz_cmd)
    ld.add_action(gps_cmd)

    return ld
