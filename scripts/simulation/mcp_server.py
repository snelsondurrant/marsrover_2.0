#!/usr/bin/env python3
# Created by Nelson Durrant, July 2025

"""
I created this MCP server as an quick experiment for a summer internship project I was working on,
but it's actually turned out pretty well haha. Right now it's configured to just work with the
simulator, but could be adapted to run on the base station (with an internet connection or some local
model work -- ollama?). It does incur a significant amount of processing overhead. Not sure how much
of our real competition performance we trust off-loading to an LLM, but if future teams are curious
or just want to play around with a simple way to hook up an LLM to our workflow or testing, here's
the steps to get started:

1.  On a Windows computer, install Claude Desktop and follow the instructions to set up a
    MCP connection using this link: https://modelcontextprotocol.io/quickstart/user. Instead
    of the filesystem tools, add the following config to 'claude_desktop_config.json':

    "mcpServers": {
        "roverMcpServerExperimental": {
            "type": "command",
            "command": "docker",
            "args": [
                "exec", "-i", "marsrover-ct", "bash", "-c", 
                "source /home/marsrover-docker/rover_ws/install/setup.bash && uv run /home/marsrover-docker/scripts/simulation/mcp_server.py"
            ]
        }
    }

2.  Make sure the Docker container is up and running in the background and relaunch Claude Desktop.
    It should now automatically connect to the MCP server when the container is running, although
    the exposed tools currently really won't do much unless the simulation is running as well.

    NOTE: As of now, Claude Desktop only attempts to reconnect to the MCP server when it is first
    launched. If you kill the server somehow, close Claude Desktop (maybe check task manager to make
    sure it's really closed!), ensure the container is running, and relaunch it again.

3.  Launch the simulation and try out some commands! Here's a few suggestions to get started:
    - "What city is the rover in? What is it doing rn?"
    - "Describe to me what the rover is seeing right now. What's around it?"
    - "Add a new GPS waypoint to the GUI. Is a task running already?"
    - "Navigate to a waypoint 20m north of the rover and look for an ArUco tag."
    - "Drive around and look for anything that could vaguely be a water bottle."

4.  Add some functionality! I tried to make the MCP server structure as flexible as possible, so
    you should be able to add new tools or modify existing ones pretty easily -- just follow the
    pattern and flow of the existing code in this file. Here's the docs tho:
    https://modelcontextprotocol.io/introduction

NOTE: As of now this also works (besides the image-based tools) with Gemini CLI as well, and I'm
sure other multi-modal LLM providers will add MCP support in the near future -- it's a pretty
quickly-growing standard. Claude Desktop is the best I've found so far tho.
https://github.com/google-gemini/gemini-cli?tab=readme-ov-file#quickstart

Nelson Durrant, July 2025
"""

import base64
import io
from typing import Any, Dict, Optional, Union

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from aruco_opencv_msgs.msg import ArucoDetection
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Vector3
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from mcp.server.fastmcp import FastMCP, Image
from nav_msgs.msg import OccupancyGrid, Odometry
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import Imu, NavSatFix
from std_srvs.srv import SetBool, Trigger
from zed_msgs.msg import ObjectsStamped
from rover_interfaces.srv import (
    GetWaypoints,
    AddWaypoint,
    RemoveWaypoint,
    IsTaskRunning,
    SendWaypoint,
    GetFeedback,
)


mcp = FastMCP("rover-mcp-server-experimental")


#################################
# General ROS 2 Helper Fuctions #
#################################


def _ros_image_to_mcp_image(ros_image: RosImage, bridge: CvBridge) -> Image:
    """Converts a sensor_msgs/Image to an MCP Image."""
    try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="rgb8")
        pil_image = PILImage.fromarray(cv_image)
        buffered = io.BytesIO()
        pil_image.save(buffered, format="JPEG")
        raw_image_bytes = buffered.getvalue() 
        return Image(data=raw_image_bytes, format="jpeg")
    except Exception as e:
        return f"[ERROR: Image Conversion] Failed to convert ROS Image to MCP Image. Details: {e}"


def _occupancy_grid_to_mcp_image(grid_msg: OccupancyGrid) -> Image:
    """Converts a nav_msgs/OccupancyGrid to an MCP Image."""
    width, height = grid_msg.info.width, grid_msg.info.height
    costmap_data = np.array(grid_msg.data, dtype=np.int8).reshape((height, width))
    pixels = np.zeros((height, width, 3), dtype=np.uint8)
    pixels[costmap_data == -1] = [128, 128, 128]  # Gray for unknown
    pixels[costmap_data == 0] = [255, 255, 255]  # White for free space
    pixels[costmap_data == 100] = [0, 0, 0]  # Black for lethal obstacle

    # Color scaled costs
    mask_cost = (costmap_data > 0) & (costmap_data < 100)
    normalized_costs = costmap_data[mask_cost] / 99.0
    colors = (plt.cm.plasma(normalized_costs)[:, :3] * 255).astype(np.uint8)
    pixels[mask_cost] = colors

    # Draw green cross at the center to represent the rover
    center_x, center_y = width // 2, height // 2
    cross_size = 1
    pixels[center_y - cross_size : center_y + cross_size + 1, center_x] = [0, 255, 0]
    pixels[center_y, center_x - cross_size : center_x + cross_size + 1] = [0, 255, 0]

    img = PILImage.fromarray(np.flipud(pixels), mode="RGB")
    buffered = io.BytesIO()
    img.save(buffered, format="PNG")
    raw_image_bytes = buffered.getvalue()
    return Image(data=raw_image_bytes, format="png")


class Simple_ROS2_MCP_Node(Node):
    """
    A simple ROS2 node that acts as a gateway for MCP commands.

    :author: Nelson Durrant (w Gemini 2.5 Pro)
    :date: July 2025
    """

    def __init__(self):
        super().__init__("simple_ros2_mcp_node")
        self.bridge = CvBridge()
        self.publisher_cache: Dict[str, Publisher] = {}

    def _spin_for_future(self, future: Future, timeout_sec: Optional[float] = None):
        """Spins the node until a future is complete."""
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

    def get_message(
        self, topic: str, msg_type: Any, qos: QoSProfile, timeout_sec: float
    ) -> Optional[Any]:
        """Subscribes to a topic and retrieves a single message."""
        future = Future()
        sub = self.create_subscription(
            msg_type,
            topic,
            lambda msg: not future.done() and future.set_result(msg),
            qos,
        )
        try:
            self._spin_for_future(future, timeout_sec=timeout_sec)
            return future.result() if future.done() else None
        finally:
            self.destroy_subscription(sub)

    def publish_message(
        self,
        topic_name: str,
        msg_type: Any,
        message: Any,
        publish_count: int = 3,
        qos_profile: QoSProfile = QoSProfile(depth=10),
    ) -> str:
        """Publishes a pre-made message to a topic a specified number of times."""
        publisher = self.publisher_cache.get(topic_name)
        if not publisher:
            publisher = self.create_publisher(msg_type, topic_name, qos_profile)
            self.publisher_cache[topic_name] = publisher

        try:
            for _ in range(publish_count):
                publisher.publish(message)
                rclpy.spin_once(self, timeout_sec=0.05)
            return f"Successfully published to '{topic_name}' {publish_count} times."
        except Exception as e:
            return f"[ERROR: Publisher] Failed to publish to the '{topic_name}' topic. Details: {e}"

    def call_service(
        self, srv_name: str, srv_type: Any, request: Any, timeout_sec: float = 5.0
    ) -> Optional[Any]:
        """Calls a ROS2 service and waits for the response."""
        client = self.create_client(srv_type, srv_name)
        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().error(f"Service '{srv_name}' not available.")
                return None

            future = client.call_async(request)
            self._spin_for_future(future, timeout_sec=timeout_sec)

            if future.done():
                return future.result()
            else:
                self.get_logger().error(f"Service call to '{srv_name}' timed out.")
                return None
        finally:
            self.destroy_client(client)


ROS_NODE: Optional[Simple_ROS2_MCP_Node] = None


########################
# Mars Rover MCP Tools #
########################

# Rover Sensor Tools


@mcp.tool(name="rover_sensors_getGpsFix")
def rover_sensors_getGpsFix(timeout_sec: float = 5.0) -> str:
    """
    Retrieves the latest raw GPS data (a NavSatFix message) from the rover.

    This is the primary tool for getting the rover's absolute position in the world,
    providing latitude, longitude, and altitude.

    Args:
        timeout_sec: Max time in seconds to wait for a message.

    Returns:
        The raw NavSatFix message as a string, or an error if it times out.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    msg = ROS_NODE.get_message("/gps/fix", NavSatFix, QoSProfile(depth=1), timeout_sec)
    return (
        str(msg)
        if msg
        else "[ERROR: rover_sensors_getGpsFix] Timed out waiting for a message on the '/gps/fix' topic. Is the GPS publisher running?"
    )


@mcp.tool(name="rover_sensors_getImuData")
def rover_sensors_getImuData(timeout_sec: float = 5.0) -> str:
    """
    Retrieves the latest Inertial Measurement Unit (IMU) data.

    IMU data is crucial for understanding the rover's orientation and motion.
    It includes orientation (as a quaternion), angular velocity (rotation speed),
    and linear acceleration (movement speed).

    Args:
        timeout_sec: Max time in seconds to wait for a message.

    Returns:
        The raw IMU message as a string, or an error if it times out.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    msg = ROS_NODE.get_message("/imu", Imu, QoSProfile(depth=1), timeout_sec)
    return (
        str(msg)
        if msg
        else "[ERROR: rover_sensors_getImuData] Timed out waiting for a message on the '/imu' topic. Is the IMU publisher running?"
    )


@mcp.tool(name="rover_sensors_getCameraImage")
def rover_sensors_getCameraImage(timeout_sec: float = 10.0) -> Image:
    """
    Retrieves a single color image from the rover's forward-facing camera.

    Use this tool to visually inspect the rover's immediate environment. If this
    tool times out, it likely means the camera topic is not being published.

    Args:
        timeout_sec: Time to wait for a single image frame to be published.

    Returns:
        An MCP Image object containing the captured picture, or an error message.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    ros_image = ROS_NODE.get_message(
        "/intel_realsense_r200_depth/image_raw",
        RosImage,
        QoSProfile(depth=1),
        timeout_sec,
    )
    if not ros_image:
        return "[ERROR: rover_sensors_getCameraImage] Timed out waiting for an image on the '/intel_realsense_r200_depth/image_raw' topic. Is the camera node publishing?"
    return _ros_image_to_mcp_image(ros_image, ROS_NODE.bridge)


# Rover Data Tools


@mcp.tool(name="rover_data_getLocalOdometry")
def rover_data_getLocalOdometry(timeout_sec: float = 5.0) -> str:
    """
    Retrieves the rover's local odometry data.

    This data estimates the rover's position and orientation relative to its
    starting point by fusing wheel encoder and IMU data. It is useful for tracking
    short-term movement but is prone to drift over time and distance.

    Args:
        timeout_sec: Max time in seconds to wait for a message.

    Returns:
        The Odometry message from /odometry/local as a string, or an error.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    msg = ROS_NODE.get_message(
        "/odometry/local", Odometry, QoSProfile(depth=1), timeout_sec
    )
    return (
        str(msg)
        if msg
        else "[ERROR: rover_data_getLocalOdometry] Timed out waiting for a message on the '/odometry/local' topic. Is the odometry node running?"
    )


@mcp.tool(name="rover_data_getGlobalOdometry")
def rover_data_getGlobalOdometry(timeout_sec: float = 5.0) -> str:
    """
    Retrieves the rover's global odometry data.

    This is the most accurate source for the rover's absolute position and heading
    in the world frame (East-North-Up), as it is typically fused from GPS and IMU
    data, correcting for drift.

    Args:
        timeout_sec: Max time in seconds to wait for a message.

    Returns:
        The Odometry message from /odometry/global as a string, or an error.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    msg = ROS_NODE.get_message(
        "/odometry/global", Odometry, QoSProfile(depth=1), timeout_sec
    )
    return (
        str(msg)
        if msg
        else "[ERROR: rover_data_getGlobalOdometry] Timed out waiting for a message on the '/odometry/global' topic. Is the sensor fusion (e.g., EKF) node running?"
    )


@mcp.tool(name="rover_data_getArucoDetections")
def rover_data_getArucoDetections(timeout_sec: int = 5) -> str:
    """
    Retrieves the latest detected ArUco (visual fiducial) markers.

    If this tool times out, it is likely because ArUco detection has not been enabled.

    Args:
        timeout_sec: Time to wait for the latest ArUco marker detections.

    Returns:
        A string containing the latest ArUco detection message, or an error if it times out.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    msg = ROS_NODE.get_message(
        "/aruco_detections", ArucoDetection, QoSProfile(depth=1), timeout_sec
    )
    return (
        str(msg)
        if msg
        else "[ERROR: rover_data_getArucoDetections] Timed out waiting for a message on the '/aruco_detections' topic. Is ArUco detection enabled?"
    )


@mcp.tool(name="rover_data_getObjectDetections")
def rover_data_getObjectDetections(timeout_sec: int = 5) -> str:
    """
    Retrieves the latest object detections from the camera's neural network.

    If this tool times out, it is likely because object detection has not been enabled.

    Args:
        timeout_sec: Time to wait for the latest object detections.

    Returns:
        A string containing the latest ObjectsStamped message, or an error if it times out.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    msg = ROS_NODE.get_message(
        "/zed/zed_node/obj_det/objects",
        ObjectsStamped,
        QoSProfile(depth=1),
        timeout_sec,
    )
    return (
        str(msg)
        if msg
        else "[ERROR: rover_data_getObjectDetections] Timed out waiting for a message on the '/zed/zed_node/obj_det/objects' topic. Is ZED object detection enabled?"
    )


@mcp.tool(name="rover_data_getLocalCostmapImage")
def rover_data_getLocalCostmapImage(timeout_sec: int = 10) -> Image:
    """
    Generates an image of the rover's local costmap for immediate obstacle avoidance.

    **IMPORTANT!** The image legend is as follows:
    - **Green Cross**: Rover's current position (at the center).
    - **Black**: Lethal obstacle (collision is certain).
    - **Yellow to Blue**: Inflated obstacle cost (Yellow=High Cost, Blue=Low Cost).
    - **White**: Free space (safe to traverse).
    - **Gray**: Unknown space.

    Args:
        timeout_sec: Time to wait for the costmap data to be published.

    Returns:
        An MCP Image object containing the costmap image, or an error message if it times out.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    msg = ROS_NODE.get_message(
        "/local_costmap/costmap", OccupancyGrid, qos, timeout_sec
    )
    if not msg:
        return "[ERROR: rover_data_getLocalCostmapImage] Timed out waiting for data on the '/local_costmap/costmap' topic. Is the navigation stack running?"
    return _occupancy_grid_to_mcp_image(msg)


@mcp.tool(name="rover_data_getGlobalCostmapImage")
def rover_data_getGlobalCostmapImage(timeout_sec: int = 10) -> Image:
    """
    Generates an image of the rover's global costmap for long-range path planning.

    **IMPORTANT!** The image legend is as follows:
    - **Green Cross**: Rover's current position.
    - **Black**: Lethal obstacle (collision is certain).
    - **Yellow to Blue**: Inflated obstacle cost (Yellow=High Cost, Blue=Low Cost).
    - **White**: Free space (safe to traverse).
    - **Gray**: Unknown space.

    Args:
        timeout_sec: Time to wait for the costmap data to be published.

    Returns:
        An MCP Image object containing the costmap image, or an error message if it times out.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    msg = ROS_NODE.get_message(
        "/global_costmap/costmap", OccupancyGrid, qos, timeout_sec
    )
    if not msg:
        return "[ERROR: rover_data_getGlobalCostmapImage] Timed out waiting for data on the '/global_costmap/costmap' topic. Is the navigation stack running?"
    return _occupancy_grid_to_mcp_image(msg)


# Rover Mode Tools


@mcp.tool(name="rover_mode_enableArucoDetection")
def rover_mode_enableArucoDetection(enable: bool, timeout_sec: float = 5.0) -> str:
    """
    Enables or disables the ArUco marker detection node.

    **IMPORTANT!** Disable when not in use to avoid unnecessary resource usage.

    This function controls a lifecycle node. Enabling activates it, and disabling
    deactivates it. This is necessary before using rover_data_getArucoDetections.

    Args:
        enable: Set to True to enable detection, False to disable.
        timeout_sec: Max time to wait for the service response.

    Returns:
        The full service response as a string.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."

    req = ChangeState.Request()
    if enable:
        req.transition.id = Transition.TRANSITION_ACTIVATE
    else:
        req.transition.id = Transition.TRANSITION_DEACTIVATE

    response = ROS_NODE.call_service(
        "/aruco_tracker/change_state", ChangeState, req, timeout_sec
    )

    if response:
        return str(response)
    return "[ERROR: rover_mode_enableArucoDetection] Service call to '/aruco_tracker/change_state' timed out or the service is unavailable. Is the ArUco tracker node running?"


@mcp.tool(name="rover_mode_enableObjectDetection")
def rover_mode_enableObjectDetection(enable: bool, timeout_sec: float = 5.0) -> str:
    """
    Enables or disables the ZED camera's object detection module.

    **IMPORTANT!** Disable when not in use to avoid unnecessary resource usage.

    This must be enabled before using rover_data_getObjectDetections.

    Args:
        enable: Set to True to enable detection, False to disable.
        timeout_sec: Max time to wait for the service response.

    Returns:
        The full service response as a string.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."

    req = SetBool.Request(data=enable)

    response = ROS_NODE.call_service(
        "/zed/zed_node/enable_obj_det", SetBool, req, timeout_sec
    )

    if response:
        return str(response)
    return "[ERROR: rover_mode_enableObjectDetection] Service call to '/zed/zed_node/enable_obj_det' timed out or the service is unavailable. Is the ZED node running?"


# Rover Actuator Tools


@mcp.tool(name="rover_actuators_driveRover")
def rover_actuators_driveRover(linear_x: float, angular_z: float) -> str:
    """
    Sends a non-blocking velocity command to the rover.

    This command is published multiple times for reliability. It will run until a
    new command is published or explicitly stopped. To stop the rover, you MUST call this
    function again with both linear_x and angular_z set to 0.

    **IMPORTANT!** This will not execute as planned if the rover is currently
    executing an autonomy task. You must cancel the task first using the
    rover_autonomy_cancelTask tool.

    Args:
        linear_x: The forward (positive) or backward (negative) velocity in m/s.
        angular_z: The counter-clockwise (positive) or clockwise (negative)
                   angular velocity in rad/s.

    Returns:
        A confirmation that the command was published.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."

    # Directly create and populate the ROS2 message object
    twist_msg = Twist()
    twist_msg.linear = Vector3(x=linear_x, y=0.0, z=0.0)
    twist_msg.angular = Vector3(x=0.0, y=0.0, z=angular_z)

    return ROS_NODE.publish_message(
        topic_name="/cmd_vel", msg_type=Twist, message=twist_msg, publish_count=3
    )


# Rover GUI Tools


@mcp.tool(name="rover_gui_getAllWaypoints")
def rover_gui_getAllWaypoints() -> str:
    """
    Retrieves a list of all waypoints currently loaded in the Autonomy GUI.

    Use this tool to inspect the current mission plan before execution.

    Returns:
        The full service response as a string.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    req = GetWaypoints.Request()
    response = ROS_NODE.call_service("/autonomy_gui/get_waypoints", GetWaypoints, req)
    if response:
        return str(response)
    return "[ERROR: rover_gui_getAllWaypoints] Service call to '/autonomy_gui/get_waypoints' timed out or the service is unavailable. Is the autonomy GUI running?"


@mcp.tool(name="rover_gui_addWaypoint")
def rover_gui_addWaypoint(
    name: str,
    waypoint_type: str,
    latitude: float,
    longitude: float,
    tag_id: Optional[int] = None,
    object_name: Optional[str] = None,
) -> str:
    """
    Adds a new waypoint to the mission plan in the Autonomy GUI.

    This allows for building a sequence of tasks for the rover to execute.
    Waypoints must have a unique name.

    Args:
        name: The unique name for the waypoint (e.g., 'crater_edge_1').
        waypoint_type: The type of waypoint. Must be one of 'gps', 'aruco', or 'obj'.
        latitude: The latitude coordinate for the waypoint.
        longitude: The longitude coordinate for the waypoint.
        tag_id: Required if waypoint_type is 'aruco'. The ID of the ArUco tag (1, 2, or 3).
        object_name: Required if waypoint_type is 'obj'. The name of the object ('mallet' or 'bottle').

    Returns:
        The full service response as a string.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."

    req = AddWaypoint.Request()
    req.leg.name = name
    req.leg.type = waypoint_type
    req.leg.latitude = latitude
    req.leg.longitude = longitude

    if waypoint_type == "aruco":
        if tag_id is None:
            return "[ERROR: rover_gui_addWaypoint] Invalid arguments. The 'tag_id' parameter is required for 'aruco' waypoint types."
        req.leg.tag_id = tag_id
    elif waypoint_type == "obj":
        if object_name is None:
            return "[ERROR: rover_gui_addWaypoint] Invalid arguments. The 'object_name' parameter is required for 'obj' waypoint types."
        req.leg.object = object_name

    response = ROS_NODE.call_service("/autonomy_gui/add_waypoint", AddWaypoint, req)
    if response:
        return str(response)
    return "[ERROR: rover_gui_addWaypoint] Service call to '/autonomy_gui/add_waypoint' timed out or the service is unavailable. Is the autonomy GUI running?"


@mcp.tool(name="rover_gui_removeWaypoint")
def rover_gui_removeWaypoint(name: str) -> str:
    """
    Removes a waypoint from the mission plan by its unique name.

    Args:
        name: The name of the waypoint to remove.

    Returns:
        The full service response as a string.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    req = RemoveWaypoint.Request(name=name)
    response = ROS_NODE.call_service(
        "/autonomy_gui/remove_waypoint", RemoveWaypoint, req
    )
    if response:
        return str(response)
    return "[ERROR: rover_gui_removeWaypoint] Service call to '/autonomy_gui/remove_waypoint' timed out or the service is unavailable. Is the autonomy GUI running?"


@mcp.tool(name="rover_gui_setTerrainPlanning")
def rover_gui_setTerrainPlanning(enable: bool) -> str:
    """
    Enables or disables terrain-aided path planning for autonomy tasks.

    When enabled, the rover uses a pre-loaded terrain map to plan more efficient paths.

    Args:
        enable: Set to True to enable, False to disable.

    Returns:
        The full service response as a string.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    req = SetBool.Request(data=enable)
    response = ROS_NODE.call_service("/autonomy_gui/set_terrain_planning", SetBool, req)
    if response:
        return str(response)
    return "[ERROR: rover_gui_setTerrainPlanning] Service call to '/autonomy_gui/set_terrain_planning' timed out or the service is unavailable. Is the autonomy GUI running?"


# Rover Autonomy Tools


@mcp.tool(name="rover_autonomy_isTaskRunning")
def rover_autonomy_isTaskRunning() -> str:
    """
    Checks if an autonomy task (a mission plan) is currently being executed by the rover.

    Returns:
        The full service response as a string.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    req = IsTaskRunning.Request()
    response = ROS_NODE.call_service(
        "/autonomy_gui/is_task_running", IsTaskRunning, req
    )
    if response:
        return str(response)
    return "[ERROR: rover_autonomy_isTaskRunning] Service call to '/autonomy_gui/is_task_running' timed out or the service is unavailable. Is the autonomy GUI running?"


@mcp.tool(name="rover_autonomy_sendWaypoint")
def rover_autonomy_sendWaypoint(name: str) -> str:
    """
    Commands the rover to execute a single waypoint from the mission plan.

    This will fail if a task is already running.

    Args:
        name: The unique name of the waypoint to execute.

    Returns:
        The full service response as a string.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    req = SendWaypoint.Request(name=name)
    response = ROS_NODE.call_service("/autonomy_gui/send_waypoint", SendWaypoint, req)
    if response:
        return str(response)
    return "[ERROR: rover_autonomy_sendWaypoint] Service call to '/autonomy_gui/send_waypoint' timed out or the service is unavailable. Is the autonomy GUI running?"


@mcp.tool(name="rover_autonomy_sendAllWaypoints")
def rover_autonomy_sendAllWaypoints() -> str:
    """
    Commands the rover to execute the entire mission plan (all loaded waypoints).

    This will fail if a task is already running.

    Returns:
        The full service response as a string.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    req = Trigger.Request()
    response = ROS_NODE.call_service("/autonomy_gui/send_all_waypoints", Trigger, req)
    if response:
        return str(response)
    return "[ERROR: rover_autonomy_sendAllWaypoints] Service call to '/autonomy_gui/send_all_waypoints' timed out or the service is unavailable. Is the autonomy GUI running?"


@mcp.tool(name="rover_autonomy_getFeedback")
def rover_autonomy_getFeedback() -> str:
    """
    Retrieves the full text log from the Autonomy GUI's 'Task Feedback' display.

    Returns:
        The full service response as a string.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    req = GetFeedback.Request()
    response = ROS_NODE.call_service("/autonomy_gui/get_feedback", GetFeedback, req)
    if response:
        return str(response)
    return "[ERROR: rover_autonomy_getFeedback] Service call to '/autonomy_gui/get_feedback' timed out or the service is unavailable. Is the autonomy GUI running?"


@mcp.tool(name="rover_autonomy_cancelTask")
def rover_autonomy_cancelTask() -> str:
    """
    Immediately cancels any autonomy task that is currently running.

    If no task is running, this will report a failure.

    Returns:
        The full service response as a string.
    """
    if not ROS_NODE:
        return "[ERROR: System] The ROS 2 gateway node is not running. Please restart the server."
    req = Trigger.Request()
    response = ROS_NODE.call_service("/autonomy_gui/cancel_task", Trigger, req)
    if response:
        return str(response)
    return "[ERROR: rover_autonomy_cancelTask] Service call to '/autonomy_gui/cancel_task' timed out or the service is unavailable. Is the autonomy GUI running?"


def main():
    global ROS_NODE
    rclpy.init()
    ROS_NODE = Simple_ROS2_MCP_Node()
    try:
        mcp.run(transport="stdio")
    except KeyboardInterrupt:
        pass
    finally:
        ROS_NODE.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
