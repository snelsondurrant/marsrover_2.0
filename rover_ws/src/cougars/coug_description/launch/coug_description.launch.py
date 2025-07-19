# Created by Nelson Durrant, July 2025
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = FindPackageShare(package="coug_description").find("coug_description")
    urdf_path = PathJoinSubstitution([pkg_share, "urdf/couguv.urdf.xacro"])

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {
                # Use the xacro command to process the URDF file
                "robot_description": ParameterValue(
                    Command(["xacro ", urdf_path]), value_type=str
                ),
            }
        ],
    )

    start_joint_state_publisher_cmd = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)

    return ld
