# Created by Nelson Durrant, Feb 2025
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation time"
    )

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            launch_ros.actions.LifecycleNode(
                # https://github.com/fictionlab/ros_aruco_opencv
                # This only launches in real life
                package="aruco_opencv",
                executable="aruco_tracker",
                name="aruco_tracker",
                namespace="",
                output="screen",
                parameters=[
                    {
                        "cam_base_topic": "zed/zed_node/rgb_gray/image_rect_gray",
                        "image_is_rectified": True,
                        "marker_size": 0.2,
                        "use_sim_time": use_sim_time,
                    }
                ],
                condition=UnlessCondition(use_sim_time),
            ),
            launch_ros.actions.LifecycleNode(
                # https://github.com/fictionlab/ros_aruco_opencv
                # This only launches in simulation
                package="aruco_opencv",
                executable="aruco_tracker",
                name="aruco_tracker",
                namespace="",
                output="screen",
                parameters=[
                    {
                        "cam_base_topic": "intel_realsense_r200_depth/image_raw",
                        "marker_size": 0.2,
                        "use_sim_time": use_sim_time,
                    }
                ],
                condition=IfCondition(use_sim_time),
            ),
        ]
    )
