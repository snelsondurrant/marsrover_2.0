#!/usr/bin/env python3
# Created by Nelson Durrant (w Gemini 2.5 Pro), July 2025

# I created this MCP server mostly as an experiment for another project I was working on and
# wouldn't recommend it for real competition usage. Right now it's configured to just work
# with the simulation (mostly). If future teams are curious tho or just want to play around
# with a simple way to hook up an LLM to our workflow, here's the steps to get started:
#
# 1. OUTSIDE OF THE DOCKER CONTAINER, set up the Gemini CLI by following the instructions provided
#    here: https://github.com/google-gemini. Run 'gemini' to make sure it works.
#
# 2. Add the following config to '~/.gemini/settings.json':
#
#    "mcpServers": {
#      "roverExperimental": {
#        "type": "command",
#        "command": "docker",
#        "args": ["exec", "-i", "marsrover-ct", "bash", "-c",
#          "cd /home/marsrover-docker/scripts/simulation/ && source /home/marsrover-docker/rover_ws/install/setup.bash && uv run mcp_server.py && pkill -f mcp_server.py"
#         ]
#       }
#     }
#
# 3. Make sure the Docker container is up and running in the background and relaunch the Gemini
#    CLI to apply the new settings. It should now automatically connect to the MCP server when the
#    container is running, although the exposed tools really won't do much unless the simulation
#    is running as well.
#
# 4. Try out some commands! Here's a few suggestions to get started:
#    - "What city is the rover in? Do a Google search. What is it doing there?"
#    - "Describe to me what the rover is seeing right now"
#    - "Drive the rover in a circle with a radius of 5 meters"
#    - "Send a GPS waypoint task to the rover within 40 meters of its current position"

from mcp.server.fastmcp import FastMCP, Image
import subprocess
from subprocess import TimeoutExpired
import os
import tempfile
import shutil

mcp = FastMCP("rover-mcp-server")


# Helper function to run subprocess commands and handle output
def _run_ros2_command(command: list[str], timeout: int | None = None) -> str:
    """
    A private helper to execute ROS2 commands, returning stdout or stderr.
    """
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=False,
            timeout=timeout,
        )
        if result.returncode != 0:
            if result.stderr:
                return f"ERROR: {result.stderr.strip()}"
            return f"ERROR: Command failed with return code {result.returncode}"
        return result.stdout.strip()
    except TimeoutExpired:
        return f"ERROR: Command timed out after {timeout} seconds."


# --- Sensor Data Tools ---
@mcp.tool()
def get_rover_gps_fix(timeout_sec: int = 10) -> str:
    """
    Retrieves the latest raw GPS fix from the sensor.

    Args:
        timeout_sec (int): The maximum time in seconds to wait for a message.
                           This typically should be 10.

    Returns:
        str: The NavSatFix message from the /gps/fix topic or an error.
    """
    command = [
        "ros2",
        "topic",
        "echo",
        "--no-arr",
        "--no-str",
        "--once",
        "/gps/fix",
    ]
    return _run_ros2_command(command, timeout=timeout_sec)


@mcp.tool()
# NOTE: See below for why this is commented out.
# def get_rover_camera_image(capture_duration_sec: int = 10) -> Image:
def get_rover_camera_image(capture_duration_sec: int = 10) -> str:
    """
    Retrieves a single image from the rover's camera.

    Args:
        capture_duration_sec (int): The number of seconds to let the image
                                    capture node run. This typically should be 10.

    Returns:
        (NOT CURRENTLY) Image: An MCP Image object containing the captured picture.
        str: A message indicating the image was captured and saved, or an error.
    """
    with tempfile.TemporaryDirectory() as temp_dir:
        command = [
            "ros2",
            "run",
            "image_view",
            "extract_images",
            "--ros-args",
            "--remap",
            "image:=/intel_realsense_r200_depth/image_raw",
        ]

        try:
            subprocess.run(
                command,
                cwd=temp_dir,  # Save images into our safe temporary directory
                capture_output=True,
                text=True,
                check=False,
                timeout=capture_duration_sec,
            )
        except subprocess.TimeoutExpired:
            pass  # Use the timeout to stop the image capture

        saved_files = sorted([f for f in os.listdir(temp_dir) if f.endswith(".jpg")])
        if not saved_files:
            raise FileNotFoundError("Image capture failed. Is the topic publishing?")

        # NOTE: I think the below is the best solution, but as of July 2025, returning an MCP
        # Image type is not supported by the Gemini CLI. The issue has been reported and added
        # to the development queue, so hopefully it will be in the future.
        #
        # captured_image_path = os.path.join(temp_dir, saved_files[0])
        # final_file = tempfile.NamedTemporaryFile(suffix=".jpg", delete=False)
        # final_file.close()
        # os.rename(captured_image_path, final_file.name)
        #
        # return Image(path=final_file.name)

        # Until the above is resolved, this is a temporary working solution.
        shutil.copy(
            os.path.join(temp_dir, saved_files[0]),
            os.path.join(os.path.dirname(__file__), "captured_image.jpg"),
        )
        return (
            "Image captured and saved as 'captured_image.jpg' in 'scripts/simulation/'.\n\n"
            + "Until the Gemini CLI project releases MCP Image processing functionality, the user needs to manually include the generated image in their prompt using the '@' syntax for it to be processed correctly."
        )


@mcp.tool()
def get_rover_odometry_local(timeout_sec: int = 10) -> str:
    """
    Retrieves a snapshot of the rover's local odometry data.

    This is typically the wheel odometry fused with IMU data, representing the rover's motion
    relative to its starting point.

    Args:
        timeout_sec (int): The maximum time in seconds to wait for a message. This typically
                           should be 10.

    Returns:
        str: The odometry message from the /odometry/local topic or an error.
    """
    command = [
        "ros2",
        "topic",
        "echo",
        "--no-arr",
        "--no-str",
        "--once",
        "/odometry/local",
    ]
    return _run_ros2_command(command, timeout=timeout_sec)


@mcp.tool()
def get_rover_aruco_detections(timeout_sec: int = 10) -> str:
    """
    Retrieves the latest ArUco tag detections.

    If no tags are detected, this tool will time out after the specified duration.

    Args:
        timeout_sec (int): The maximum time in seconds to wait for a message. This typically
                           should be 10.

    Returns:
        str: The ArUco detection message or an error.
    """
    command = [
        "ros2",
        "topic",
        "echo",
        "--no-arr",
        "--no-str",
        "--once",
        "/aruco_detections",
    ]
    return _run_ros2_command(command, timeout=timeout_sec)


@mcp.tool()
def get_rover_obj_detections(timeout_sec: int = 10) -> str:
    """
    Retrieves the latest object detections from the ZED camera.

    If no objects are detected, this tool will time out after the specified duration.

    Args:
        timeout_sec (int): The maximum time in seconds to wait for a message. This typically
                           should be 10.

    Returns:
        str: The object detection message or an error.
    """
    command = [
        "ros2",
        "topic",
        "echo",
        "--no-arr",
        "--no-str",
        "--once",
        "/zed/zed_node/obj_det/objects",
    ]
    return _run_ros2_command(command, timeout=timeout_sec)


# --- Rover Movement Tool ---
@mcp.tool()
def move_rover(linear_x: float, angular_z: float, timeout_sec: int = 10) -> str:
    """
    Sends a single velocity command to the rover.

    The rover will continue moving at this velocity until a new command is sent. To stop the rover,
    call this function with both linear_x and angular_z set to 0.

    IMPORTANT! Check the rover's odometry data after calling this tool to ensure the command was
    received. Don't assume the rover will move just because this tool returns successfully.

    Args:
        linear_x (float): The forward (positive) or backward (negative)
                          velocity in m/s.
        angular_z (float): The counter-clockwise (positive) or clockwise (negative) angular
                           velocity in rad/s.
        timeout_sec (int): The maximum time in seconds to wait for a message. This typically
                           should be 10.

    Returns:
        str: A confirmation that the command was sent or an error.
    """
    command = [
        "ros2",
        "topic",
        "pub",
        "--once",
        "/cmd_vel",
        "geometry_msgs/msg/Twist",
        f"{{linear: {{x: {linear_x}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {angular_z}}}}}",
    ]
    return _run_ros2_command(command, timeout=timeout_sec)


# --- Autonomy Tasking Tool ---
@mcp.tool()
def send_autonomy_task(
    name: str,
    type: str,
    latitude: float,
    longitude: float,
    enable_terrain: bool = True,
    tag_id: int = 0,
    object_name: str = "",
) -> str:
    """
    Builds a single autonomy task from user inputs and sends it to the rover's action server.

    If the user's request for a task is ambiguous, ask for clarification on the specifics
    (type, coordinates, etc.) instead of inventing them.

    This tool will run to completion before a response is returned.

    Args:
        name (str): The name for the waypoint or task leg.
        type (str): The type of waypoint. Must be one of 'gps', 'aruco', or 'obj'.
        latitude (float): The latitude coordinate of the waypoint. Check to make sure
                          it's not an absurd distance from the rover's current position.
        longitude (float): The longitude coordinate of the waypoint. Check to make sure
                           it's not an absurd distance from the rover's current position.
        enable_terrain (bool): Flag to enable or disable terrain-aware path planning.
        tag_id (int): The ArUco tag ID. This is only used if the task type is 'aruco'.
                      Must be one of 1, 2, or 3.
        object_name (str): The name of the object to detect. This is only used if the
                           task type is 'obj'. Must be one of 'mallet' or 'bottle'.

    Returns:
        str: The result from the action server, including feedback, or an error message.
    """
    valid_types = ["gps", "aruco", "obj"]
    if type not in valid_types:
        return (
            f"ERROR: Invalid task type '{type}'. Please use one of the "
            f"following: {valid_types}."
        )

    valid_objects = ["mallet", "bottle"]
    if type == "obj" and object_name not in valid_objects:
        return (
            f"ERROR: Invalid object name '{object_name}'. Please use one of the "
            f"following: {valid_objects}."
        )

    valid_tag_ids = [1, 2, 3]
    if type == "aruco" and tag_id not in valid_tag_ids:
        return (
            f"ERROR: Invalid tag ID '{tag_id}'. Please use one of the "
            f"following: {valid_tag_ids}."
        )

    leg_yaml = f"""{{
        name: "{name}", type: "{type}",
        latitude: {latitude}, longitude: {longitude},
        tag_id: {tag_id if type == 'aruco' else 0},
        object: "{object_name if type == 'obj' else ''}"
    }}"""

    goal_yaml = f"""{{
        enable_terrain: {'true' if enable_terrain else 'false'},
        legs: [{leg_yaml.strip()}]
    }}"""

    command = [
        "ros2",
        "action",
        "send_goal",
        "-f",
        "/exec_autonomy_task",
        "rover_interfaces/action/AutonomyTask",
        goal_yaml,
    ]
    return _run_ros2_command(command)


if __name__ == "__main__":
    mcp.run(transport="stdio")
