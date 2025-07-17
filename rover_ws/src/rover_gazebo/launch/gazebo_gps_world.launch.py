# Created by Nelson Durrant, Feb 2025
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    gazebo_dir = get_package_share_directory("rover_gazebo")
    launch_dir = os.path.join(gazebo_dir, "launch")
    world = os.path.join(gazebo_dir, "worlds", "sonoma_raceway.world")

    models_dir = os.path.join(gazebo_dir, "models")
    models_dir += (
        os.pathsep
        + f"/opt/ros/{os.getenv('ROS_DISTRO')}/share/turtlebot3_gazebo/models"
    )
    set_gazebo_model_path_cmd = None

    if "GAZEBO_MODEL_PATH" in os.environ:
        gazebo_model_path = os.environ["GAZEBO_MODEL_PATH"] + os.pathsep + models_dir
        set_gazebo_model_path_cmd = SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH", gazebo_model_path
        )
    else:
        set_gazebo_model_path_cmd = SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH", models_dir
        )

    set_tb3_model_cmd = SetEnvironmentVariable("TURTLEBOT3_MODEL", "waffle")


    spawn_rover_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=["-topic", "/robot_description",
                   "-entity", "rover",
                   "-x", "-54.5",
                   "-y", "127.8",
                   "-z", "0.3",
                   "-Y", "0.0"]
    )

    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            "gzserver",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world,
        ],
        cwd=[launch_dir],
        output="both",
    )

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=["gzclient"], cwd=[launch_dir], output="both"
    )

    # Create the launch description and populate
    ld = LaunchDescription(
        [
            Node(
                package="rover_gazebo",
                executable="sim_obj_detect",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "enable_mallet": True,
                        "enable_bottle": False,
                    }
                ],
            )
        ]
    )

    # Set gazebo up to find models properly
    ld.add_action(set_gazebo_model_path_cmd)
    ld.add_action(set_tb3_model_cmd)

    # simulator launch
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_rover_cmd)

    return ld
