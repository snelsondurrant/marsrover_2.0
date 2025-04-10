# Created by Nelson Durrant, Feb 2025
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from nav2_common.launch import RewrittenYaml
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():

    use_rviz = LaunchConfiguration("use_rviz")
    use_mapviz = LaunchConfiguration("use_mapviz")
    sim_mode = LaunchConfiguration("sim_mode")

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="False", description="Whether to start RVIZ"
    )

    declare_use_mapviz_cmd = DeclareLaunchArgument(
        "use_mapviz", default_value="False", description="Whether to start mapviz"
    )

    declare_sim_mode_cmd = DeclareLaunchArgument(
        "sim_mode",
        default_value="False",
        description="Whether to start in simulation mode",
    )

    # Get the package directories
    bringup_dir = get_package_share_directory("nav2_bringup")
    nav_dir = get_package_share_directory("rover_navigation")
    gui_dir = get_package_share_directory("rover_gui")
    loc_dir = get_package_share_directory("rover_localization")
    perception_dir = get_package_share_directory("rover_perception")
    gz_dir = get_package_share_directory("rover_gazebo")
    description_dir = get_package_share_directory("rover_description")
    ublox_dir = get_package_share_directory("ublox_read_2")
    unitree_dir = get_package_share_directory("unitree_lidar_ros2")

    # Get the launch directories
    nav_launch_dir = os.path.join(nav_dir, "launch")
    loc_launch_dir = os.path.join(loc_dir, "launch")
    gui_launch_dir = os.path.join(gui_dir, "launch")
    perception_launch_dir = os.path.join(perception_dir, "launch")
    gz_launch_dir = os.path.join(gz_dir, "launch")
    description_launch_dir = os.path.join(description_dir, "launch")
    ublox_launch_dir = os.path.join(ublox_dir, "launch")

    # Get the params directories
    nav_params_dir = os.path.join(nav_dir, "config")
    sim_nav2_params = os.path.join(nav_params_dir, "sim_navigation_params.yaml")
    nav2_params = os.path.join(nav_params_dir, "navigation_params.yaml")
    sim_configured_params = RewrittenYaml(
        source_file=sim_nav2_params, root_key="", param_rewrites="", convert_types=True
    )
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_launch_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={
            "use_sim_time": sim_mode,
        }.items(),
    )

    gazebo_cmd = IncludeLaunchDescription(
        # This only launches in simulation
        PythonLaunchDescriptionSource(
            os.path.join(gz_launch_dir, "gazebo_gps_world.launch.py")
        ),
        condition=IfCondition(sim_mode),
    )

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(loc_launch_dir, "dual_ekf_navsat.launch.py")
        ),
        launch_arguments={
            "use_sim_time": sim_mode,
        }.items(),
    )

    sim_navigation2_cmd = IncludeLaunchDescription(
        # This only launches in simulation
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        condition=IfCondition(sim_mode),
        launch_arguments={
            "use_sim_time": sim_mode,
            "params_file": sim_configured_params,
            "autostart": "True",
        }.items(),
    )

    navigation2_cmd = IncludeLaunchDescription(
        # This only launches in real life
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        condition=UnlessCondition(sim_mode),
        launch_arguments={
            "use_sim_time": sim_mode,
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gui_launch_dir, "rviz.launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "use_sim_time": sim_mode,
        }.items(),
    )

    mapviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gui_launch_dir, "mapviz.launch.py")),
        condition=IfCondition(use_mapviz),
        launch_arguments={
            "use_sim_time": sim_mode,
        }.items(),
    )

    gui_cmd = IncludeLaunchDescription(
        # This only launches in simulation
        PythonLaunchDescriptionSource(
            os.path.join(gui_launch_dir, "autonomy_gui.launch.py")
        ),
        condition=IfCondition(sim_mode),
        launch_arguments={
            "use_sim_time": sim_mode,
        }.items(),
    )

    aruco_opencv_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_launch_dir, "aruco_opencv.launch.py")
        ),
        launch_arguments={
            "use_sim_time": sim_mode,
        }.items(),
    )

    gps_cmd = IncludeLaunchDescription(
        # This only launches in real life
        XMLLaunchDescriptionSource(os.path.join(ublox_launch_dir, "rover_launch.xml")),
        condition=UnlessCondition(sim_mode),
    )

    lidar_cmd = IncludeLaunchDescription(
        # This only launches in real life
        PythonLaunchDescriptionSource(os.path.join(unitree_dir, "launch.py")),
        condition=UnlessCondition(sim_mode),
    )

    state_machine_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, "state_machine.launch.py")
        ),
        launch_arguments={
            "use_sim_time": sim_mode,
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_sim_mode_cmd)

    # robot state publisher launch
    ld.add_action(description_cmd)

    # simulation launch
    ld.add_action(gazebo_cmd)

    # robot localization launch
    ld.add_action(robot_localization_cmd)

    # navigation2 launch
    ld.add_action(navigation2_cmd)
    ld.add_action(sim_navigation2_cmd)

    # viz launch
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(declare_use_mapviz_cmd)
    ld.add_action(mapviz_cmd)
    ld.add_action(gui_cmd)

    # custom launch
    ld.add_action(aruco_opencv_cmd)
    # ld.add_action(gps_cmd)
    # ld.add_action(lidar_cmd)
    ld.add_action(state_machine_cmd)

    return ld
