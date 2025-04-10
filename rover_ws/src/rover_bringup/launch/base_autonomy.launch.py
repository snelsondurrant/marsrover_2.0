# Created by Nelson Durrant, Mar 2025
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node


def generate_launch_description():

    use_rviz = LaunchConfiguration("use_rviz")
    use_mapviz = LaunchConfiguration("use_mapviz")

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="False", description="Whether to start RVIZ"
    )

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        "use_mapviz", default_value="False", description="Whether to start mapviz"
    )

    # Get the launch directories
    gui_dir = get_package_share_directory("rover_gui")
    gui_launch_dir = os.path.join(gui_dir, "launch")
    ublox_dir = get_package_share_directory("ublox_read_2")
    ublox_launch_dir = os.path.join(ublox_dir, "launch")

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gui_launch_dir, "rviz.launch.py")),
        condition=IfCondition(use_rviz),
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gui_launch_dir, "mapviz.launch.py")),
        condition=IfCondition(use_mapviz),
    )

    gui_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gui_launch_dir, "autonomy_gui.launch.py")),
    )

    gps_cmd = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(ublox_launch_dir, "base_launch.xml")),
    )

    joy_node_cmd = Node(
        # https://docs.ros.org/en/humble/p/joy/
        package="joy",
        executable="joy_node",
        name="joy_node_base",
        output="screen",
    )

    ld = LaunchDescription()

    # viz launch
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(mapviz_cmd)
    ld.add_action(gui_cmd)
    # ld.add_action(gps_cmd)
    ld.add_action(joy_node_cmd)

    return ld
