# Created by Nelson Durrant, Mar 2025
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation time"
    )

    # Get the launch directory
    pkg_share = FindPackageShare(package='rover_description').find('rover_description')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf/rover.urdf.xacro'])

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {
                # Use the xacro command to process the URDF file
                "robot_description": ParameterValue(Command(['xacro ', urdf_path]), value_type=str),
                "use_sim_time": use_sim_time
            }
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # robot state publisher launch
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
