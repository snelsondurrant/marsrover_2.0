#!/usr/bin/env python3
# Created by Nelson Durrant (w Gemini 2.5 Pro), July 2025

"""
I created this MCP server mostly as an experiment for another project I was working on and
wouldn't recommend it for real competition usage. Right now it's configured to just work
with the simulation (mostly). If future teams are curious tho or just want to play around
with a simple way to hook up an LLM to our workflow, here's the steps to get started:

1. On a Windows computer, install Claude Desktop and follow the instructions to set up a
   MCP connection using this link: https://modelcontextprotocol.io/quickstart/user. Instead
   of the filesystem tools, add the following config to 'claude_desktop_config.json':

   "mcpServers": {
     "roverExperimental": {
       "type": "command",
       "command": "docker",
       "args": ["exec", "-i", "marsrover-ct", "bash", "-c",
         "cd /home/marsrover-docker/scripts/simulation/ && source /home/marsrover-docker/rover_ws/install/setup.bash && uv run mcp_server.py && pkill -f mcp_server.py"
       ]
     }
   }

3. Make sure the Docker container is up and running in the background and relaunch Claude Desktop.
   It should now automatically connect to the MCP server when the container is running, although
   the exposed tools really won't do much unless the simulation is running as well.

   NOTE: As of now, Claude Desktop only attempts to reconnect to the MCP server when it is first
   launched. If you lose connection, close Claude Desktop (maybe check task manager to make
   sure it's really closed!), ensure the container is running, and relaunch it again.

4. Try out some commands! Here's a few suggestions to get started:
   - "What city is the rover in? What is it doing there?"
   - "Describe to me what the rover is seeing right now"
   - "Drive the rover in a circle with a radius of 5 meters"
   - "Send a GPS waypoint task to the rover within 40 meters of its current position"

NOTE: This also works (besides the image-based tools) with Gemini CLI currently, and I'm sure
other multi-modal LLM providers will add MCP support in the future.
https://github.com/google-gemini/gemini-cli?tab=readme-ov-file#quickstart
"""

import base64
import io
import threading
import time
import uuid
from typing import Any, Dict, Literal, Optional

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from aruco_opencv_msgs.msg import ArucoDetection
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Vector3
from mcp.server.fastmcp import FastMCP, Image
from nav_msgs.msg import OccupancyGrid, Odometry
from PIL import Image as PILImage
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rover_interfaces.action import AutonomyTask
from rover_interfaces.msg import AutonomyLeg
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import Imu, NavSatFix
from zed_msgs.msg import ObjectsStamped

# --- MCP Server Initialization ---
mcp = FastMCP("rover-mcp-server-unified")

# --- Constants ---
VALID_AUTONOMY_TASK_TYPES = ["gps", "aruco", "obj"]
VALID_OBJECT_NAMES = ["mallet", "bottle"]
VALID_ARUCO_TAG_IDS = [1, 2, 3]


# --- Unified ROS Gateway Node ---
class MCP_ROS_Gateway_Node(Node):
    """
    Acts as a singleton gateway to the ROS2 ecosystem.

    Author: Nelson Durrant (w Gemini 2.5 Pro)
    Date: July 2025

    This single, persistent node runs in a background thread and handles all
    ROS2 communications. It manages stateful operations, such as the single-task
    autonomy client, and caches communication clients (e.g., publishers) for
    performance. This architecture prevents ROS context conflicts and provides an
    efficient, centralized point of interaction for all MCP tools.
    """

    def __init__(self):
        super().__init__("mcp_ros_gateway_node")
        # --- State and Communication Clients (initialized on demand) ---
        self.active_task_id: Optional[str] = None
        self.active_tasks: Dict[str, Dict[str, Any]] = {}
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.publisher_cache: Dict[str, Publisher] = {}
        self._action_client: Optional[ActionClient] = None
        self._action_server_name: Optional[str] = None

    # --- Internal Communication Client Getters ---
    def _get_publisher(self, topic_name: str) -> Publisher:
        """
        Internal: Retrieves a publisher from cache or creates a new one.
        This caching avoids the performance cost of repeatedly creating publishers.
        """
        if topic_name not in self.publisher_cache:
            self.publisher_cache[topic_name] = self.create_publisher(
                Twist, topic_name, 10
            )
        return self.publisher_cache[topic_name]

    def _get_action_client(self, server_name: str) -> ActionClient:
        """
        Internal: Initializes the action client on its first use (lazy initialization).
        Enforces a single action server name for the node's lifetime to simplify state.
        """
        with self.lock:
            if self._action_client is None:
                self.get_logger().info(f"Creating action client for '{server_name}'")
                self._action_client = ActionClient(self, AutonomyTask, server_name)
                self._action_server_name = server_name
            elif self._action_server_name != server_name:
                self.get_logger().error(
                    f"Node is already bound to action server '{self._action_server_name}'!"
                )
        return self._action_client

    def _wait_for_message(
        self, topic: str, msg_type: Any, qos: QoSProfile, timeout: float
    ) -> Optional[Any]:
        """
        Internal: Creates a temporary subscriber to wait for a single message.
        This is the core of on-demand data fetching; it is non-blocking for the
        main background spin loop.
        """
        future = Future()
        sub = self.create_subscription(
            msg_type,
            topic,
            lambda msg: not future.done() and future.set_result(msg),
            qos,
        )
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            return future.result()
        finally:
            self.destroy_subscription(sub)

    # --- Public Methods for MCP Tools ---
    def get_single_message(self, topic: str, msg_type: Any, timeout_sec: float) -> str:
        msg = self._wait_for_message(topic, msg_type, QoSProfile(depth=1), timeout_sec)
        return str(msg) if msg else f"ERROR: Timed out waiting for message on {topic}."

    def get_camera_image(self, topic: str, timeout_sec: float) -> Image:
        ros_image = self._wait_for_message(
            topic, RosImage, QoSProfile(depth=1), timeout_sec
        )
        if not ros_image:
            return f"ERROR: Timed out waiting for an image on {topic}. The camera may not be publishing."
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
            pil_image = PILImage.fromarray(cv_image)
            buffered = io.BytesIO()
            pil_image.save(buffered, format="JPEG")
            img_bytes = base64.b64encode(buffered.getvalue())
            return Image(data=img_bytes, format="jpeg")
        except Exception as e:
            return f"ERROR: Failed to convert image. Details: {e}"

    def get_costmap_image(self, topic_name: str, timeout_sec: float) -> Image:
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        msg = self._wait_for_message(topic_name, OccupancyGrid, qos, timeout_sec)
        if not msg:
            return f"ERROR: Timed out waiting for costmap data on '{topic_name}'."

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
        pixels[center_y - cross_size : center_y + cross_size, center_x] = [
            0,
            255,
            0,
        ]  # Green vertical line
        pixels[center_y, center_x - cross_size : center_x + cross_size] = [
            0,
            255,
            0,
        ]  # Green horizontal line
        img = PILImage.fromarray(np.flipud(pixels), mode="RGB")
        buffered = io.BytesIO()
        img.save(buffered, format="PNG")
        img_bytes = base64.b64encode(buffered.getvalue())
        return Image(data=img_bytes, format="png")

    def publish_move_command(self, topic_name: str, linear_x: float, angular_z: float):
        """
        Publishes a Twist message multiple times for reliability.
        """
        publisher = self._get_publisher(topic_name)
        linear_vec = Vector3(x=linear_x, y=0.0, z=0.0)
        angular_vec = Vector3(x=0.0, y=0.0, z=angular_z)
        twist_msg = Twist(linear=linear_vec, angular=angular_vec)
        for _ in range(3):
            publisher.publish(twist_msg)
            time.sleep(0.05)

    def _feedback_callback(self, task_id: str):
        def feedback_cb(feedback_msg):
            with self.lock:
                if task_id in self.active_tasks:
                    self.active_tasks[task_id]["feedback"] = feedback_msg.feedback

        return feedback_cb

    def start_task(self, server_name: str, goal_msg):
        client = self._get_action_client(server_name)
        with self.lock:
            if self.active_task_id:
                return "ERROR: Another task is already in progress. Wait or cancel it."
        if not client.wait_for_server(timeout_sec=5.0):
            return f"ERROR: Action server '{server_name}' not available."
        task_id = str(uuid.uuid4())
        send_goal_future = client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback(task_id)
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return "ERROR: Goal was rejected by the server."
        with self.lock:
            self.active_task_id = task_id
            self.active_tasks[task_id] = {
                "goal_handle": goal_handle,
                "feedback": None,
                "result_future": goal_handle.get_result_async(),
            }
        return "Task started successfully."

    def get_feedback(self):
        with self.lock:
            if not self.active_task_id:
                return "ERROR: No active task."
            return self.active_tasks[self.active_task_id].get(
                "feedback", "No feedback yet."
            )

    def get_result(self):
        with self.lock:
            if not self.active_task_id:
                return "ERROR: No active task."
            task_id = self.active_task_id
            result_future = self.active_tasks[task_id]["result_future"]
        if result_future.done():
            result = result_future.result().result
            with self.lock:
                del self.active_tasks[task_id]
                self.active_task_id = None
            return f"Task completed with result: {result}"
        return "Task is still in progress."

    def cancel_task(self):
        with self.lock:
            if not self.active_task_id:
                return "ERROR: No active task."
            task_id = self.active_task_id
            goal_handle = self.active_tasks[task_id]["goal_handle"]
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)
        with self.lock:
            del self.active_tasks[task_id]
            self.active_task_id = None
        return f"Cancel request sent. The system is ready for a new task."


# --- Global Node Instance ---
ROS_NODE: Optional[MCP_ROS_Gateway_Node] = None


# --- Sensor Data Tools ---
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
    return ROS_NODE.get_single_message("/gps/fix", NavSatFix, timeout_sec)


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
    return ROS_NODE.get_single_message("/imu", Imu, timeout_sec)


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


@mcp.tool(name="rover_sensors_getDepthCameraImage")
def get_rover_depth_camera_image(timeout_sec: float = 10.0) -> Image:
    """
    Retrieves a single depth image from the rover's forward-facing camera.

    This provides a 3D view of the scene. The result is a grayscale image where
    pixel intensity corresponds to distance (brighter pixels are closer).

    Args:
        timeout_sec: Time to wait for a single image frame to be published.

    Returns:
        An MCP Image object containing the captured depth image.
    """
    if not ROS_NODE:
        return "ERROR: ROS Node not initialized."
    return ROS_NODE.get_camera_image(
        "/intel_realsense_r200_depth/depth/image_raw", timeout_sec
    )


@mcp.tool(name="rover_sensors_getLocalOdometry")
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
    return ROS_NODE.get_single_message("/odometry/local", Odometry, timeout_sec)


@mcp.tool(name="rover_sensors_getGlobalOdometry")
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
    return ROS_NODE.get_single_message("/odometry/global", Odometry, timeout_sec)


@mcp.tool(name="rover_sensors_getArucoDetections")
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
    return ROS_NODE.get_single_message("/aruco_detections", ArucoDetection, timeout_sec)


@mcp.tool(name="rover_sensors_getObjectDetections")
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
    return ROS_NODE.get_single_message(
        "/zed/zed_node/obj_det/objects", ObjectsStamped, timeout_sec
    )


@mcp.tool(name="rover_sensors_getLocalCostmapImage")
def get_rover_costmap_local_image(timeout_sec: int = 10) -> Image:
    """
    Generates an image of the rover's local costmap for immediate obstacle avoidance.

    **IMPORTANT!** The image legend is as follows:
    - **Green Cross**: Rover's current position (at the center).
    - **Black**: Lethal obstacle (collision is certain).
    - **Yellow to Blue**: Inflated obstacle cost (Yellow=High Cost, Blue=Low Cost).
    - **White**: Free space (safe to traverse).
    - **Gray**: Unknown space.
    """
    if not ROS_NODE:
        return "ERROR: ROS Node not initialized."
    return ROS_NODE.get_costmap_image("/local_costmap/costmap", timeout_sec)


@mcp.tool(name="rover_sensors_getGlobalCostmapImage")
def get_rover_costmap_global_image(timeout_sec: int = 10) -> Image:
    """
    Generates an image of the rover's global costmap for long-range path planning.

    **IMPORTANT!** The image legend is as follows:
    - **Green Cross**: Rover's current position.
    - **Black**: Lethal obstacle (collision is certain).
    - **Yellow to Blue**: Inflated obstacle cost (Yellow=High Cost, Blue=Low Cost).
    - **White**: Free space (safe to traverse).
    - **Gray**: Unknown space.
    """
    if not ROS_NODE:
        return "ERROR: ROS Node not initialized."
    return ROS_NODE.get_costmap_image("/global_costmap/costmap", timeout_sec)


# --- Rover Movement Tool ---
@mcp.tool(name="rover_motion_move")
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


# --- Autonomy Tasking Tools ---
@mcp.tool(name="rover_autonomy_startTask")
def start_autonomy_task(
    name: str,
    type: Literal["gps", "aruco", "obj"],
    latitude: float,
    longitude: float,
    enable_terrain: bool = True,
    tag_id: int = 1,
    object_name: str = "mallet",
) -> str:
    """
    Starts an autonomy task. This will navigate the rover to a specified waypoint
    or search for a specific object or ArUco tag.

    IMPORTANT! Ask the user for clarification on the task type and parameters if
    they are not clear.

    This call is non-blocking and reserves the action client for the task. It will
    fail if another task is already in progress. After calling this, use
    `getTaskFeedback` to monitor progress and `getTaskResult` to confirm
    completion, which frees the client for a new task.

    Args:
        name: A descriptive name for the task leg (e.g., "Navigate to science area").
        type: The type of waypoint. Must be 'gps', 'aruco', or 'obj'.
        latitude: The latitude coordinate for the goal.
        longitude: The longitude coordinate for the goal.
        enable_terrain: If True, use terrain-aware path planning.
        tag_id: The ArUco tag ID to search for. Only used if type is 'aruco'.
        object_name: The object to search for. Only used if type is 'obj'.

    Returns:
        A success or error message.
    """
    if not ROS_NODE:
        return "ERROR: ROS Node not initialized."
    if type not in VALID_AUTONOMY_TASK_TYPES:
        return f"ERROR: Invalid type '{type}'."
    if type == "obj" and object_name not in VALID_OBJECT_NAMES:
        return f"ERROR: Invalid object '{object_name}'."
    if type == "aruco" and tag_id not in VALID_ARUCO_TAG_IDS:
        return f"ERROR: Invalid tag ID '{tag_id}'."

    leg = AutonomyLeg(
        name=name,
        type=type,
        latitude=latitude,
        longitude=longitude,
        tag_id=tag_id,
        object=object_name,
    )
    goal_msg = AutonomyTask.Goal(enable_terrain=enable_terrain, legs=[leg])
    return ROS_NODE.start_task("/exec_autonomy_task", goal_msg)


@mcp.tool(name="rover_autonomy_getTaskFeedback")
def get_task_feedback() -> str:
    """
    Gets the latest feedback for the currently active autonomy task.

    Feedback includes progress updates, status messages, and any
    intermediate results. This is useful for monitoring the task's progress.

    Returns:
        A string containing the latest feedback or an error if no task is active.
    """
    if not ROS_NODE:
        return "ERROR: ROS Node not initialized."
    return str(ROS_NODE.get_feedback())


@mcp.tool(name="rover_autonomy_getTaskResult")
def get_task_result() -> str:
    """
    Requests the final result of the currently active autonomy task.

    If the task is complete, this returns the final outcome and releases the
    system lock, allowing a new task to be started. If the task is still running,
    it will return a status update.

    Returns:
        A string indicating the final result, a progress update, or an error.
    """
    if not ROS_NODE:
        return "ERROR: ROS Node not initialized."
    return ROS_NODE.get_result()


@mcp.tool(name="rover_autonomy_cancelTask")
def cancel_task() -> str:
    """
    Cancels the currently active autonomy task.

    This immediately terminates the active task and makes the system available
    for a new task.

    Returns:
        A confirmation or error message.
    """
    if not ROS_NODE:
        return "ERROR: ROS Node not initialized."
    return ROS_NODE.cancel_task()


# --- Main Execution ---
def main():
    """Initializes the unified ROS Gateway Node and starts the MCP server."""
    global ROS_NODE
    rclpy.init()
    ROS_NODE = MCP_ROS_Gateway_Node()
    ros_thread = threading.Thread(target=rclpy.spin, args=(ROS_NODE,), daemon=True)
    ros_thread.start()
    ROS_NODE.get_logger().info(
        "Unified ROS Gateway Node started and spinning in background."
    )

    try:
        mcp.run(transport="stdio")
    except KeyboardInterrupt:
        pass
    finally:
        ROS_NODE.get_logger().info("Shutting down MCP server.")
        rclpy.shutdown()
        ros_thread.join()


if __name__ == "__main__":
    main()