#!/usr/bin/env python3
# Created by Nelson Durrant, July 2025

"""
I created this MCP server as an quick experiment for a summer internship project I was working on,
but it's actually turned out pretty well haha. Right now it's configured to just work with the
simulator, but could be adapted to run on the base station (with an internet connection or some local
model work). Not sure how much of our task performance we trust offloading to an LLM, but if future
teams are curious or just want to play around with a simple way to hook up an LLM to our workflow
or testing, here's the steps to get started:

1.  On a Windows computer, install Claude Desktop and follow the instructions to set up a
    MCP connection using this link: https://modelcontextprotocol.io/quickstart/user. Instead
    of the filesystem tools, add the following config to 'claude_desktop_config.json':

    "mcpServers": {
        "roverExperimental": {
            "type": "command",
            "command": "docker",
            "args": ["exec", "-i", "marsrover-ct", "bash", "-c", "cd /home/marsrover-docker/scripts/simulation/ && source /home/marsrover-docker/rover_ws/install/setup.bash && uv run mcp_server.py && pkill -f mcp_server.py"]
        }
    }

3.  Make sure the Docker container is up and running in the background and relaunch Claude Desktop.
    It should now automatically connect to the MCP server when the container is running, although
    the exposed tools currently really won't do much unless the simulation is running as well.

    NOTE: As of now, Claude Desktop only attempts to reconnect to the MCP server when it is first
    launched. If you kill the server somehow, close Claude Desktop (maybe check task manager to make
    sure it's really closed!), ensure the container is running, and relaunch it again.

4.  Launch the simulation and try out some commands! Here's a few suggestions to get started:
    - "What city is the rover in? What is it doing there?"
    - "Describe to me what the rover is seeing right now."
    - "Enable aruco detection on the rover and tell me if you see any tags."
    - "Drive the rover in a circle and report what obstacles it identifies."
    - "Look for anything that could pass for a water bottle and navigate the rover towards it."

5.  Add some functionality! I tried to make the MCP server structure as flexible as possible, so 
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
from std_srvs.srv import SetBool
from zed_msgs.msg import ObjectsStamped

mcp = FastMCP("rover-mcp-server-unified")


class MCP_ROS_Gateway_Node(Node):
    """
    A simple ROS node that acts as a gateway for MCP commands.

    :author: Nelson Durrant (w Google Gemini 2.5 Pro)
    :date: July 2025
    """

    def __init__(self):
        super().__init__("mcp_ros_gateway_node")
        self.bridge = CvBridge()
        self.publisher_cache: Dict[str, Publisher] = {}

    def _spin_for_future(self, future: Future, timeout_sec: Optional[float] = None):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

    def _get_raw_message(
        self, topic: str, qos: QoSProfile, msg_type: Any, timeout_sec: float
    ) -> Optional[Any]:
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

    def call_service(
        self, srv_name: str, srv_type: Any, request: Any, timeout_sec: float = 5.0
    ) -> Optional[Any]:
        """
        Calls a ROS service and waits for the result.
        """
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

    def get_single_message_as_str(
        self, topic: str, msg_type: Any, timeout_sec: float
    ) -> str:
        msg = self._get_raw_message(topic, QoSProfile(depth=1), msg_type, timeout_sec)
        return (
            str(msg) if msg else f"ERROR: Timed out waiting for message on '{topic}'."
        )

    def get_camera_image(self, topic: str, timeout_sec: float) -> Union[Image, str]:
        ros_image = self._get_raw_message(
            topic, QoSProfile(depth=1), RosImage, timeout_sec
        )
        if not ros_image:
            return f"ERROR: Timed out waiting for an image on '{topic}'."
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
            pil_image = PILImage.fromarray(cv_image)
            buffered = io.BytesIO()
            pil_image.save(buffered, format="JPEG")
            img_bytes = base64.b64encode(buffered.getvalue())
            return Image(data=img_bytes, format="jpeg")
        except Exception as e:
            return f"ERROR: Failed to convert image. Details: {e}"

    def get_costmap_image(self, topic: str, timeout_sec: float) -> Union[Image, str]:
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        msg = self._get_raw_message(topic, qos, OccupancyGrid, timeout_sec)
        if not msg:
            return f"ERROR: Timed out waiting for costmap data on '{topic}'."

        # Convert OccupancyGrid to an image
        width, height = msg.info.width, msg.info.height
        costmap_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        pixels = np.zeros((height, width, 3), dtype=np.uint8)
        pixels[costmap_data == -1] = [128, 128, 128]
        pixels[costmap_data == 0] = [255, 255, 255]
        pixels[costmap_data == 100] = [0, 0, 0]
        mask_cost = (costmap_data > 0) & (costmap_data < 100)
        normalized_costs = costmap_data[mask_cost] / 99.0
        colors = (plt.cm.plasma(normalized_costs)[:, :3] * 255).astype(np.uint8)
        pixels[mask_cost] = colors
        center_x, center_y = width // 2, height // 2
        cross_size = 3
        pixels[center_y - cross_size : center_y + cross_size + 1, center_x] = [
            0,
            255,
            0,
        ]
        pixels[center_y, center_x - cross_size : center_x + cross_size + 1] = [
            0,
            255,
            0,
        ]
        img = PILImage.fromarray(np.flipud(pixels), mode="RGB")
        buffered = io.BytesIO()
        img.save(buffered, format="PNG")
        img_bytes = base64.b64encode(buffered.getvalue())
        return Image(data=img_bytes, format="png")

    def publish_move_command(self, topic_name: str, linear_x: float, angular_z: float):
        publisher = self.publisher_cache.get(topic_name)
        if not publisher:
            publisher = self.create_publisher(Twist, topic_name, 10)
            self.publisher_cache[topic_name] = publisher

        twist_msg = Twist(linear=Vector3(x=linear_x), angular=Vector3(z=angular_z))
        for _ in range(3):
            publisher.publish(twist_msg)
            rclpy.spin_once(self, timeout_sec=0.05)


ROS_NODE: Optional[MCP_ROS_Gateway_Node] = None


########################
# Mars Rover MCP Tools #
########################


@mcp.tool(name="rover_mode_enableArucoDetection")
def enable_aruco_detection(enable: bool, timeout_sec: float = 5.0) -> str:
    """
    Enables or disables the ArUco marker detection node.

    **IMPORTANT!** Disable when not in use to avoid unnecessary resource usage.

    This function controls a lifecycle node. Enabling activates it, and disabling
    deactivates it. This is necessary before using get_rover_aruco_detections.

    Args:
        enable: Set to True to enable detection, False to disable.
        timeout_sec: Max time to wait for the service response.

    Returns:
        A success or error message string.
    """
    if not ROS_NODE:
        return "ERROR: ROS Node not initialized."

    req = ChangeState.Request()
    if enable:
        req.transition.id = Transition.TRANSITION_ACTIVATE
        action = "enable"
    else:
        req.transition.id = Transition.TRANSITION_DEACTIVATE
        action = "disable"

    response = ROS_NODE.call_service(
        "/aruco_tracker/change_state", ChangeState, req, timeout_sec
    )

    if response is None:
        return f"ERROR: Failed to {action} ArUco detection. Service call timed out or service is not available."

    if response.success:
        return f"Successfully sent request to {action} ArUco detection."
    else:
        return f"ERROR: Service reported failure on request to {action} ArUco detection."


@mcp.tool(name="rover_mode_enableObjectDetection")
def enable_object_detection(enable: bool, timeout_sec: float = 5.0) -> str:
    """
    Enables or disables the ZED camera's object detection module.

    **IMPORTANT!** Disable when not in use to avoid unnecessary resource usage.

    This must be enabled before using get_rover_obj_detections.

    Args:
        enable: Set to True to enable detection, False to disable.
        timeout_sec: Max time to wait for the service response.

    Returns:
        A success or error message string.
    """
    if not ROS_NODE:
        return "ERROR: ROS Node not initialized."

    req = SetBool.Request()
    req.data = enable
    action = "enable" if enable else "disable"

    response = ROS_NODE.call_service(
        "/zed/zed_node/enable_obj_det", SetBool, req, timeout_sec
    )

    if response is None:
        return f"ERROR: Failed to {action} object detection. Service call timed out or service is not available."

    if response.success:
        return f"Successfully sent request to {action} object detection."
    else:
        return f"ERROR: Service reported failure on request to {action} object detection."


@mcp.tool(name="rover_sensors_getGpsFix")
def get_rover_gps_fix(timeout_sec: float = 5.0) -> str:
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
        return "ERROR: ROS Node not initialized."
    return ROS_NODE.get_single_message_as_str("/gps/fix", NavSatFix, timeout_sec)


@mcp.tool(name="rover_sensors_getImuData")
def get_rover_imu_data(timeout_sec: float = 5.0) -> str:
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
        return "ERROR: ROS Node not initialized."
    return ROS_NODE.get_single_message_as_str("/imu", Imu, timeout_sec)


@mcp.tool(name="rover_sensors_getCameraImage")
def get_rover_camera_image(timeout_sec: float = 10.0) -> Image:
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
        return "ERROR: ROS Node not initialized."
    return ROS_NODE.get_camera_image(
        "/intel_realsense_r200_depth/image_raw", timeout_sec
    )


@mcp.tool(name="rover_data_getLocalOdometry")
def get_rover_odometry_local(timeout_sec: float = 5.0) -> str:
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
        return "ERROR: ROS Node not initialized."
    return ROS_NODE.get_single_message_as_str("/odometry/local", Odometry, timeout_sec)


@mcp.tool(name="rover_data_getGlobalOdometry")
def get_rover_odometry_global(timeout_sec: float = 5.0) -> str:
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
        return "ERROR: ROS Node not initialized."
    return ROS_NODE.get_single_message_as_str("/odometry/global", Odometry, timeout_sec)


@mcp.tool(name="rover_data_getArucoDetections")
def get_rover_aruco_detections(timeout_sec: int = 5) -> str:
    """
    Retrieves the latest detected ArUco (visual fiducial) markers.

    If this tool times out, it is likely because ArUco detection has not been enabled.

    Args:
        timeout_sec: Time to wait for the latest ArUco marker detections.

    Returns:
        A string containing the latest ArUco detection message, or an error if it times out.
    """
    if not ROS_NODE:
        return "ERROR: ROS Node not initialized."
    return ROS_NODE.get_single_message_as_str(
        "/aruco_detections", ArucoDetection, timeout_sec
    )


@mcp.tool(name="rover_data_getObjectDetections")
def get_rover_obj_detections(timeout_sec: int = 5) -> str:
    """
    Retrieves the latest object detections from the camera's neural network.

    If this tool times out, it is likely because object detection has not been enabled.

    Args:
        timeout_sec: Time to wait for the latest object detections.

    Returns:
        A string containing the latest ObjectsStamped message, or an error if it times out.
    """
    if not ROS_NODE:
        return "ERROR: ROS Node not initialized."
    return ROS_NODE.get_single_message_as_str(
        "/zed/zed_node/obj_det/objects", ObjectsStamped, timeout_sec
    )


@mcp.tool(name="rover_data_getLocalCostmapImage")
def get_rover_costmap_local_image(timeout_sec: int = 10) -> Image:
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
        return "ERROR: ROS Node not initialized."
    return ROS_NODE.get_costmap_image("/local_costmap/costmap", timeout_sec)


@mcp.tool(name="rover_data_getGlobalCostmapImage")
def get_rover_costmap_global_image(timeout_sec: int = 10) -> Image:
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
        return "ERROR: ROS Node not initialized."
    return ROS_NODE.get_costmap_image("/global_costmap/costmap", timeout_sec)


@mcp.tool(name="rover_actuators_driveRover")
def move_rover(linear_x: float, angular_z: float) -> str:
    """
    Sends a non-blocking velocity command to the rover.

    This command is published multiple times for reliability. It will run until a
    new command is published or explicitly stopped. To stop the rover, you MUST call this
    function again with both linear_x and angular_z set to 0.

    Args:
        linear_x: The forward (positive) or backward (negative) velocity in m/s.
        angular_z: The counter-clockwise (positive) or clockwise (negative)
                   angular velocity in rad/s.

    Returns:
        A confirmation that the command was published.
    """
    if not ROS_NODE:
        return "ERROR: ROS Node not initialized."
    ROS_NODE.publish_move_command("/cmd_vel", linear_x, angular_z)
    return f"Published Twist command: linear_x={linear_x}, angular_z={angular_z}"


def main():
    global ROS_NODE
    rclpy.init()
    ROS_NODE = MCP_ROS_Gateway_Node()
    try:
        mcp.run(transport="stdio")
    except KeyboardInterrupt:
        pass
    finally:
        ROS_NODE.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()