# Created by Nelson Durrant, Apr 2025
import launch
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="rover_gui",
                executable="autonomy_gui",
                output="screen",
            ),
        ]
    )
