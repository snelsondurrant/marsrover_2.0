# Created by Nelson Durrant, Feb 2025
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation time"
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
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
