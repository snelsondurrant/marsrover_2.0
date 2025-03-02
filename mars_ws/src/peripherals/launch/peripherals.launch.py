# Created by Nelson Durrant, Feb 2025
import launch
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="peripherals",
            executable="rover_status_listener",
            output="screen",
        ),
    ])