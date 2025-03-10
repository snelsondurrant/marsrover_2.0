# Created by Nelson Durrant, Feb 2025
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.conditions import UnlessCondition
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation time"
    )
    cam_config_path = os.path.join(
        get_package_share_directory("rover_navigation"), "config", "aruco_cam_params.yaml"
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            launch_ros.actions.Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                namespace="aruco_cam",
                output="screen",
                parameters=[cam_config_path],
                condition=UnlessCondition(use_sim_time),
            ),
            launch_ros.actions.Node(
                # https://github.com/fictionlab/ros_aruco_opencv
                package="aruco_opencv",
                executable="aruco_tracker_autostart",
                output="screen",
                parameters=[
                    {
                        "cam_base_topic": "aruco_cam/image_raw",
                        "marker_size": 0.2,
                        "use_sim_time": use_sim_time,
                    }
                ],
                remappings=[
                    ("/aruco_cam/image_raw", "/intel_realsense_r200_depth/image_raw"),
                    (
                        "/aruco_cam/camera_info",
                        "/intel_realsense_r200_depth/camera_info",
                    ),
                ],
            ),
        ]
    )
