# Created by Nelson Durrant, Feb 2025
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                # https://github.com/fictionlab/ros_aruco_opencv
                package="aruco_opencv",
                executable="aruco_tracker_autostart",
                name="aruco_tracker_autostart",
                output="screen",
                parameters=[{"cam_base_topic": "camera1/image_raw", "marker_size": 0.15, "image_is_rectified": True}],
            ),
        ]
    )
