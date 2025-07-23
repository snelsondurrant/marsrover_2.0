# Created by Nelson Durrant, July 2025
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os


def generate_launch_description():

    coug_loc_dir = get_package_share_directory("coug_localization")
    coug_loc_params_file = os.path.join(
        coug_loc_dir, "config", "localization_params.yaml"
    )

    return LaunchDescription(
        [
            # https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[coug_loc_params_file],
                remappings=[("odometry/filtered", "odometry/local")],
            ),
            # https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[coug_loc_params_file],
                remappings=[("odometry/filtered", "odometry/global")],
            ),
            # https://docs.ros.org/en/melodic/api/robot_localization/html/navsat_transform_node.html
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[coug_loc_params_file],
                remappings=[
                    ("imu/data", "heading/data"), # use the global heading from the modem
                    ("gps/fix", "gps/fix"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
            ),
            launch_ros.actions.Node(
                package="rover_localization",
                executable="sync_origin",
                name="sync_origin",
                output="screen",
                parameters=[coug_loc_params_file],
            ),
        ]
    )
