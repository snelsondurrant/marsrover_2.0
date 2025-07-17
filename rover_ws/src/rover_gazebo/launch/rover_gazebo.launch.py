# Created by Nelson Durrant, Feb 2025
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    
    gz_dir = get_package_share_directory("rover_gazebo")
    gz_launch_dir = os.path.join(gz_dir, "launch")

    world = os.path.join(gz_dir, "worlds", "sonoma_raceway.world")
    models_dir = os.path.join(gz_dir, "models")
    models_dir += os.pathsep
    set_gz_model_path_cmd = None

    if "GAZEBO_MODEL_PATH" in os.environ:
        gz_model_path = os.environ["GAZEBO_MODEL_PATH"] + os.pathsep + models_dir
        set_gz_model_path_cmd = SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH", gz_model_path
        )
    else:
        set_gz_model_path_cmd = SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH", models_dir
        )

    # Spawn the rover in the simulation
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

    # Start the Gazebo server
    start_gz_server_cmd = ExecuteProcess(
        cmd=[
            "gzserver",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world,
        ],
        cwd=[gz_launch_dir],
        output="both",
    )

    # Start the Gazebo client
    start_gz_client_cmd = ExecuteProcess(
        cmd=["gzclient"], cwd=[gz_launch_dir], output="both"
    )

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

    ld.add_action(set_gz_model_path_cmd)
    ld.add_action(start_gz_server_cmd)
    ld.add_action(start_gz_client_cmd)
    ld.add_action(spawn_rover_cmd)

    return ld
