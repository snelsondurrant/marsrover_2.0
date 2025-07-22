# Created by Nelson Durrant, July 2025
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("coug_gui")
    mapviz_params_file = os.path.join(pkg_share, "config", "mapviz_params.mvc")

    mapviz_node = Node(
        package="mapviz",
        executable="mapviz",
        name="mapviz",
        parameters=[
            {"config": mapviz_params_file}
        ],
    )

    initialize_origin_node = Node(
        package="swri_transform_util",
        executable="initialize_origin.py",
        name="initialize_origin",
        remappings=[
            ("fix", "mapviz/origin"),  # set by the 'sync_origin' node
        ],
    )

    static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="swri_transform",
        arguments=["0", "0", "0", "0", "0", "0", "map", "origin"],
    )

    return LaunchDescription([
        mapviz_node,
        initialize_origin_node,
        static_transform_node,
    ])
