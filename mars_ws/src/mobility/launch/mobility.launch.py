import launch
import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    
    
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            # https://docs.ros.org/en/iron/p/joy/
            package="joy",
            executable="joy_node",
            output="screen",
        ),
        launch_ros.actions.Node(
            # https://github.com/ros2/teleop_twist_joy
            package="teleop_twist_joy",
            executable="teleop_node",
            output="screen",
        ),
        launch_ros.actions.Node(
            package="mobility",
            executable="mobility",
            output="screen",
            parameters=[{"port": "/dev/ttyACM0", "baudrate": 9600}],
        ),
    ])