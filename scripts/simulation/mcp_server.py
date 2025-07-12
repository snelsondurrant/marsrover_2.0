#!/usr/bin/env python3
# Created by Nelson Durrant (w Gemini 2.5 Pro), July 2025

# I created this MCP server mostly as an experiment for another project I was working on and
# wouldn't recommend it for real competition usage. Right now it's configured to just work
# with the simulation (mostly). If future teams are curious tho or just want to play around
# with a simple way to hook up an LLM to our workflow, here's the steps to get started:
#
# 1. On a Windows computer, install Claude Desktop and follow the instructions to set up a
#    MCP connection using this link: https://modelcontextprotocol.io/quickstart/user. Instead
#    of the filesystem tools, add the following config to 'claude_desktop_config.json':
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
# 3. Make sure the Docker container is up and running in the background and relaunch Claude Desktop.
#    It should now automatically connect to the MCP server when the container is running, although
#    the exposed tools really won't do much unless the simulation is running as well.
#
#    NOTE: As of now, Claude Desktop only attempts to reconnect to the MCP server when it is first
#    launched. If you lose connection, close Claude Desktop (maybe check task manager to make
#    sure it's really closed!), ensure the container is running, and relaunch it again.
#
# 4. Try out some commands! Here's a few suggestions to get started:
#    - "What city is the rover in? What is it doing there?"
#    - "Describe to me what the rover is seeing right now"
#    - "Drive the rover in a circle with a radius of 5 meters"
#    - "Send a GPS waypoint task to the rover within 40 meters of its current position"
#
# NOTE: This also works (besides the image-based tools) with Gemini CLI currently, and I'm sure
# other multi-modal LLM providers will add MCP support in the future.
# https://github.com/google-gemini/gemini-cli?tab=readme-ov-file#quickstart

from mcp.server.fastmcp import FastMCP, Image
import subprocess
from subprocess import TimeoutExpired
import os
import tempfile
import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt

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
def get_rover_imu_data(timeout_sec: int = 10) -> str:
    """
    Retrieves the latest IMU data from the rover's sensors.

    Args:
        timeout_sec (int): The maximum time in seconds to wait for a message.
                           This typically should be 10.

    Returns:
        str: The IMU message from the /imu topic or an error.
    """
    command = [
        "ros2",
        "topic",
        "echo",
        "--no-arr",
        "--no-str",
        "--once",
        "/imu",
    ]
    return _run_ros2_command(command, timeout=timeout_sec)


@mcp.tool()
def get_rover_camera_image(capture_duration_sec: int = 15) -> Image:
    """
    Retrieves a single image from the rover's camera.

    Args:
        capture_duration_sec (int): The number of seconds to let the image
                                    capture command run. This typically should be 15.

    Returns:
        Image: An MCP Image object containing the captured picture.
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

        captured_image_path = os.path.join(temp_dir, saved_files[0])
        final_file = tempfile.NamedTemporaryFile(suffix=".jpg", delete=False)
        final_file.close()
        os.rename(captured_image_path, final_file.name)

        return Image(path=final_file.name)


@mcp.tool()
def get_rover_depth_camera_image(capture_duration_sec: int = 15) -> Image:
    """
    Retrieves a single depth image from the rover's camera.

    Args:
        capture_duration_sec (int): The number of seconds to let the image
                                    capture command run. This typically should be 15.

    Returns:
        Image: An MCP Image object containing the captured depth image.
    """
    with tempfile.TemporaryDirectory() as temp_dir:
        command = [
            "ros2",
            "run",
            "image_view",
            "extract_images",
            "--ros-args",
            "--remap",
            "image:=/intel_realsense_r200_depth/depth/image_raw",
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
            raise FileNotFoundError(
                "Depth image capture failed. Is the topic publishing?"
            )

        captured_image_path = os.path.join(temp_dir, saved_files[0])
        final_file = tempfile.NamedTemporaryFile(suffix=".jpg", delete=False)
        final_file.close()
        os.rename(captured_image_path, final_file.name)

        return Image(path=final_file.name)


class CostmapListener(Node):
    """
    A simple node to listen for a single OccupancyGrid message.

    NOTE: This workaround is needed bc 'ros2 topic echo' doesn't print the whole array
    for such a large message, and we need the full costmap data for processing.
    """

    def __init__(self):
        super().__init__("costmap_listener_tool")
        self.msg = None
        self.subscription = self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self.listener_callback,
            rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def listener_callback(self, msg):
        """
        Callback to store the received message.
        """
        self.get_logger().info("Received costmap message.")
        self.msg = msg


@mcp.tool()
def get_rover_costmap_local_image(timeout_sec: int = 10) -> Image:
    """
    Retrieves a single local costmap image from the rover's navigation stack with color.

    You can use this to see if there are any obstacles in the rover's immediate vicinity.

    IMPORTANT! The center of the costmap is the rover's current position. The colors represent:
    - BLACK: Lethal obstacle (collision is certain).
    - YELLOW to BLUE: Inflated obstacle cost, where yellow is higher cost (closer to an obstacle)
      and blue is lower cost.
    - WHITE: Free space (no obstacles).
    - GRAY: Unknown space.

    Args:
        timeout_sec (int): The maximum time in seconds to wait for a message.
                           This typically should be 10.

    Returns:
        Image: An MCP Image object containing the captured color costmap image.
    """
    rclpy.init()
    listener_node = CostmapListener()

    try:
        rclpy.spin_until_future_complete(
            listener_node, Future(), timeout_sec=float(timeout_sec)
        )

        if listener_node.msg is None:
            return "ERROR: Timeout waiting for costmap data."

        msg = listener_node.msg
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        costmap_data = msg.data

        listener_node.get_logger().info(
            f"Processing costmap of size {width}x{height} with resolution {resolution}..."
        )

        costmap_array = np.array(costmap_data, dtype=np.int8).reshape((height, width))

        # Create an empty 3-channel (RGB) image array
        pixels = np.zeros((height, width, 3), dtype=np.uint8)

        # Assign colors based on costmap values
        pixels[costmap_array == -1] = [128, 128, 128]  # Gray for unknown
        pixels[costmap_array == 0] = [255, 255, 255]  # White for free space
        pixels[costmap_array == 100] = [0, 0, 0]  # Black for lethal obstacles

        # Create a color gradient for inflated obstacle costs (1-99)
        mask_cost = (costmap_array > 0) & (costmap_array < 100)
        cost_values = costmap_array[mask_cost]

        # Normalize cost values to be between 0 and 1 for the colormap
        normalized_costs = cost_values / 99.0

        # Use a colormap (e.g., plasma) to get colors
        # We take the first 3 channels (R, G, B) and scale to 0-255
        colors = (plt.cm.plasma(normalized_costs)[:, :3] * 255).astype(np.uint8)
        pixels[mask_cost] = colors

        # Add a cross to indicate the rover's position
        center_x = width // 2
        center_y = height // 2
        cross_size = 10  # Size of the cross

        # Draw horizontal line
        pixels[center_y, max(0, center_x - cross_size):min(width, center_x + cross_size)] = [255, 0, 0]  # Blue
        # Draw vertical line
        pixels[max(0, center_y - cross_size):min(height, center_y + cross_size), center_x] = [255, 0, 0]  # Blue

        # Create an image from the pixel array
        img = PILImage.fromarray(np.flipud(pixels), mode="RGB")

        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as temp_file:
            temp_filename = temp_file.name
            img.save(temp_filename)
            listener_node.get_logger().info(
                f"Costmap image saved to temporary file: {temp_filename}"
            )

        return Image(path=temp_filename)

    finally:
        listener_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class GlobalCostmapListener(Node):
    """
    A simple node to listen for a single OccupancyGrid message.

    NOTE: This workaround is needed bc 'ros2 topic echo' doesn't print the whole array
    for such a large message, and we need the full costmap data for processing.
    """

    def __init__(self):
        super().__init__("costmap_listener_tool")
        self.msg = None
        self.subscription = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.listener_callback,
            rclpy.qos.QoSProfile(
                depth=1,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def listener_callback(self, msg):
        """
        Callback to store the received message.
        """
        self.get_logger().info("Received costmap message.")
        self.msg = msg


@mcp.tool()
def get_rover_costmap_global_image(timeout_sec: int = 10) -> Image:
    """
    Retrieves a single global costmap image from the rover's navigation stack with color.

    You can use this to see if there are any obstacles in the rover's global vicinity.

    IMPORTANT! The center of the costmap is the rover's current position. The colors represent:
    - BLACK: Lethal obstacle (collision is certain).
    - YELLOW to BLUE: Inflated obstacle cost, where yellow is higher cost (closer to an obstacle)
      and blue is lower cost.
    - WHITE: Free space (no obstacles).
    - GRAY: Unknown space.

    Args:
        timeout_sec (int): The maximum time in seconds to wait for a message.
                           This typically should be 10.

    Returns:
        Image: An MCP Image object containing the captured color costmap image.
    """
    rclpy.init()
    listener_node = GlobalCostmapListener()

    try:
        rclpy.spin_until_future_complete(
            listener_node, Future(), timeout_sec=float(timeout_sec)
        )

        if listener_node.msg is None:
            return "ERROR: Timeout waiting for costmap data."

        msg = listener_node.msg
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        costmap_data = msg.data

        listener_node.get_logger().info(
            f"Processing costmap of size {width}x{height} with resolution {resolution}..."
        )

        costmap_array = np.array(costmap_data, dtype=np.int8).reshape((height, width))

        # Create an empty 3-channel (RGB) image array
        pixels = np.zeros((height, width, 3), dtype=np.uint8)

        # Assign colors based on costmap values
        pixels[costmap_array == -1] = [128, 128, 128]  # Gray for unknown
        pixels[costmap_array == 0] = [255, 255, 255]  # White for free space
        pixels[costmap_array == 100] = [0, 0, 0]  # Black for lethal obstacles

        # Create a color gradient for inflated obstacle costs (1-99)
        mask_cost = (costmap_array > 0) & (costmap_array < 100)
        cost_values = costmap_array[mask_cost]

        # Normalize cost values to be between 0 and 1 for the colormap
        normalized_costs = cost_values / 99.0

        # Use a colormap (e.g., plasma) to get colors
        # We take the first 3 channels (R, G, B) and scale to 0-255
        colors = (plt.cm.plasma(normalized_costs)[:, :3] * 255).astype(np.uint8)
        pixels[mask_cost] = colors

        # Add a cross to indicate the rover's position
        center_x = width // 2
        center_y = height // 2
        cross_size = 5  # Size of the cross

        # Draw horizontal line
        pixels[center_y, max(0, center_x - cross_size):min(width, center_x + cross_size)] = [255, 0, 0]  # Blue
        # Draw vertical line
        pixels[max(0, center_y - cross_size):min(height, center_y + cross_size), center_x] = [255, 0, 0]  # Blue

        # Create an image from the pixel array
        img = PILImage.fromarray(np.flipud(pixels), mode="RGB")

        with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as temp_file:
            temp_filename = temp_file.name
            img.save(temp_filename)
            listener_node.get_logger().info(
                f"Costmap image saved to temporary file: {temp_filename}"
            )

        return Image(path=temp_filename)

    finally:
        listener_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


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
def get_rover_odometry_global(timeout_sec: int = 10) -> str:
    """
    Retrieves a snapshot of the rover's global odometry data.

    This is typically the GPS-based odometry, representing the rover's position in the world.
    The rover uses the East-North-Up (ENU) coordinate system.

    IMPORTANT! You can get the rover's global heading from this.

    Args:
        timeout_sec (int): The maximum time in seconds to wait for a message. This typically
                           should be 10.

    Returns:
        str: The odometry message from the /odometry/global topic or an error.
    """
    command = [
        "ros2",
        "topic",
        "echo",
        "--no-arr",
        "--no-str",
        "--once",
        "/odometry/global",
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
def move_rover(linear_x: float, angular_z: float) -> str:
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

    Returns:
        str: A confirmation that the command was sent or an error.
    """
    command = [
        "ros2",
        "topic",
        "pub",
        "--times",
        "5",  # Publish the command 5 times to ensure it is received
        "/cmd_vel",
        "geometry_msgs/msg/Twist",
        f"{{linear: {{x: {linear_x}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {angular_z}}}}}",
    ]
    return _run_ros2_command(command)


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
