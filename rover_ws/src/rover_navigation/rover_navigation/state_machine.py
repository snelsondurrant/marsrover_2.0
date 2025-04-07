import asyncio
import rclpy
import tf2_geometry_msgs
import tf2_ros
import time
import utm
from action_msgs.msg import GoalStatus
from aruco_opencv_msgs.msg import ArucoDetection
from builtin_interfaces.msg import Duration
from enum import Enum, auto
from geometry_msgs.msg import Pose, PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowWaypoints, Spin
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from rover_interfaces.action import AutonomyTask
from rover_interfaces.msg import AutonomyLeg
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger, SetBool
from threading import RLock
from typing import Any
from zed_msgs.msg import ObjectsStamped


from rover_navigation.utils.gps_utils import (
    latLonYaw2Geopose,
    meters2LatLon,
    latLon2Meters,
)
from rover_navigation.utils.plan_utils import (
    basicPathPlanner,  # plan a straight line between two GPS coordinates
    bruteOrderPlanner,  # use brute force to find the best order of legs
    greedyOrderPlanner,  # use a greedy algorithm to find the best order of legs
    noOrderPlanner,  # don't reorder the legs
)
from rover_navigation.utils.terrain_utils import (
    terrainPathPlanner,
    terrainOrderPlanner,
)


class State(Enum):
    INIT = auto()
    NEXT_LEG = auto()
    GPS_NAV = auto()
    SPIN_SEARCH = auto()
    NEXT_HEX = auto()
    HEX_NAV = auto()
    FOUND_NAV = auto()
    SIGNAL_SUCCESS = auto()


class Result(Enum):
    FOUND = auto()
    SUCCEEDED = auto()
    FAILED = auto()


class PatchRclpyIssue1123(ActionClient):
    """
    ActionClient patch for rclpy timing issue when multi-threading
    https://github.com/ros2/rclpy/issues/1123
    """

    _lock: RLock = None  # type: ignore

    @property
    def _cpp_client_handle_lock(self) -> RLock:
        if self._lock is None:
            self._lock = RLock()
        return self._lock

    async def execute(self, *args: Any, **kwargs: Any) -> None:
        with self._cpp_client_handle_lock:
            return await super().execute(*args, **kwargs)  # type: ignore

    def send_goal_async(self, *args: Any, **kwargs: Any) -> Future:
        with self._cpp_client_handle_lock:
            return super().send_goal_async(*args, **kwargs)

    def _cancel_goal_async(self, *args: Any, **kwargs: Any) -> Future:
        with self._cpp_client_handle_lock:
            return super()._cancel_goal_async(*args, **kwargs)

    def _get_result_async(self, *args: Any, **kwargs: Any) -> Future:
        with self._cpp_client_handle_lock:
            return super()._get_result_async(*args, **kwargs)


class StateMachine(Node):
    """
    Class for executing the autonomy task using the Nav2 stack

    Note: This is a pretty complex node. It's a hacked-together combination of the BasicNavigator
    class and our own custom state machine with a lot of multi-threading. It's easiest to think of
    as three sections with their own seperate threads: the state machine, the Nav2 BasicNavigator,
    and the ROS 2 callbacks.

    :author: Nelson Durrant
    :date: Mar 2025

    Subscribers:
    - gps/filtered (sensor_msgs/NavSatFix) [norm_callback_group]
    - aruco_detections (aruco_opencv_msgs/ArucoDetection) [norm_callback_group]
    - zed/zed_node/obj_det/objects (zed_msgs/ObjectsStamped) [norm_callback_group]
    Publishers:
    - mapviz/goal (sensor_msgs/NavSatFix)
    - mapviz/inter (sensor_msgs/NavSatFix)
    Clients:
    - trigger_teleop (std_srvs/Trigger) [norm_callback_group]
    - trigger_auto (std_srvs/Trigger) [norm_callback_group]
    - trigger_arrival (std_srvs/Trigger) [norm_callback_group]
    - zed/zed_node/enable_obj_det (std_srvs/SetBool) [norm_callback_group]
    Action Clients:
    - follow_waypoints (nav2_msgs/FollowWaypoints) [basic_nav_callback_group]
    - spin (nav2_msgs/Spin) [basic_nav_callback_group]
    - {node_name}/get_state (lifecycle_msgs/GetState) [basic_nav_callback_group] (temporary)
    Action Servers:
    - exec_autonomy_task (rover_interfaces/AutonomyTask) [action_callback_group]
    *And a tf2 buffer and listener for pose to GPS transforms
    """

    def __init__(self):

        super().__init__("state_machine")
        # self.navigator = BasicNavigator() # Don't uncomment this line
        # IMPORTANT! Simply using the BasicNavigator class causes A LOT of threading issues.
        # We've included the relevant functions from the BasicNavigator class into this class as a fix.
        # https://github.com/ros-navigation/navigation2/tree/main/nav2_simple_commander

        # Leg and order planner parameters
        self.declare_parameter("path_planner", "basicPathPlanner")
        self.declare_parameter("order_planner", "greedyOrderPlanner")
        self.path_planner = self.get_parameter("path_planner").value
        self.order_planner = self.get_parameter("order_planner").value

        # Tunable values
        self.declare_parameter("wait_time", 5)
        self.declare_parameter("update_threshold", 0.4)
        self.declare_parameter("waypoint_distance", 18.0)
        self.declare_parameter("spin_stops", 4)
        self.declare_parameter("spin_wait_time", 0.5)
        self.declare_parameter("gps_nav_timeout", 210)
        self.declare_parameter("hex_nav_timeout", 45)
        self.wait_time = self.get_parameter("wait_time").value
        self.update_threshold = self.get_parameter("update_threshold").value
        self.waypoint_distance = self.get_parameter("waypoint_distance").value
        self.spin_stops = self.get_parameter("spin_stops").value
        self.spin_wait_time = self.get_parameter("spin_wait_time").value
        self.gps_nav_timeout = self.get_parameter("gps_nav_timeout").value
        self.hex_nav_timeout = self.get_parameter("hex_nav_timeout").value

        # Assuming we can detect objects and aruco tags up to 5m away, we've determined this is the best
        # search pattern for covering the 20m radius (fastest traversal, least overlap, most coverage).
        # It could definitely be changed or tuned in the future as we get more data.
        # -->
        #           (07)
        #   (12) (06) (01) (08)
        #      (05) (00) (02)
        #   (11) (04) (03) (09)
        #           (10)
        # <--
        self.hex_coord = [
            (4.5, 7.79),
            (9.0, 0.0),
            (4.5, -7.79),
            (-4.5, -7.79),
            (-9.0, 0.0),
            (-4.5, 7.79),
            (0.0, 15.58),
            (13.5, 7.79),
            (13.5, -7.79),
            (0.0, -15.58),
            (-13.5, -7.79),
            (-13.5, 7.79),
        ]

        # Object detection dict
        self.obj_to_label = {"mallet": "Class ID: 0", "bottle": "Class ID: 1"}

        # UTM zone and hemisphere (will set on first gps fix)
        self.zone = None
        self.hemisphere = None
        self.filtered_gps = None

        # Initialize variables
        self.legs = []
        self.leg = None

        #################################
        ### ROS 2 OBJECT DECLARATIONS ###
        #################################

        # Set up a Tf2 buffer for pose to GPS transforms (aruco, object)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Callback groups (for threading)
        norm_callback_group = MutuallyExclusiveCallbackGroup()
        action_callback_group = (
            ReentrantCallbackGroup()
        )  # needed to monitor cancel requests

        # Filtered GPS location subscriber
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            "gps/filtered",
            self.gps_callback,
            10,
            callback_group=norm_callback_group,
        )
        self.gps_subscriber  # prevent unused variable warning

        # Aruco detection pose subscriber
        self.aruco_subscriber = self.create_subscription(
            ArucoDetection,
            "aruco_detections",
            self.aruco_callback,
            10,
            callback_group=norm_callback_group,
        )
        self.aruco_subscriber  # prevent unused variable warning

        # Object detection pose subscriber
        self.obj_subscriber = self.create_subscription(
            ObjectsStamped,
            "zed/zed_node/obj_det/objects",
            self.obj_callback,
            10,
            callback_group=norm_callback_group,
        )
        self.obj_subscriber  # prevent unused variable warning

        # Mapviz publishers (to show the GPS destinations in mapviz)
        self.mapviz_goal_publisher = self.create_publisher(NavSatFix, "mapviz/goal", 10)
        self.mapviz_inter_publisher = self.create_publisher(
            NavSatFix, "mapviz/inter", 10
        )

        # Client to trigger teleop state
        self.teleop_client = self.create_client(
            Trigger, "trigger_teleop", callback_group=norm_callback_group
        )
        while not self.teleop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Teleop trigger service not available, waiting again..."
            )
        self.teleop_request = Trigger.Request()

        # Client to trigger autonomy state
        self.auto_client = self.create_client(
            Trigger, "trigger_auto", callback_group=norm_callback_group
        )
        while not self.auto_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Autonomy trigger service not available, waiting again..."
            )
        self.auto_request = Trigger.Request()

        # Client to trigger arrival state
        self.arrival_client = self.create_client(
            Trigger, "trigger_arrival", callback_group=norm_callback_group
        )
        while not self.arrival_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Arrival trigger service not available, waiting again..."
            )
        self.arrival_request = Trigger.Request()

        # Client to enable/disable object detection
        self.obj_client = self.create_client(
            SetBool, "zed/zed_node/enable_obj_det", callback_group=norm_callback_group
        )
        while not self.obj_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Object detection service not available, waiting again..."
            )
        self.obj_request = SetBool.Request()

        # Action server to run the task executor
        self.action_server = ActionServer(
            self,
            AutonomyTask,
            "exec_autonomy_task",
            self.action_server_callback,
            callback_group=action_callback_group,
            cancel_callback=self.cancel_callback,
        )

        #####################################
        ### END ROS 2 OBJECT DECLARATIONS ###
        #####################################

        #######################################
        ### NAV2 BASIC NAVIGATOR BASED CODE ###
        #######################################

        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        self.basic_nav_callback_group = MutuallyExclusiveCallbackGroup()
        self.spin_client = PatchRclpyIssue1123(
            self, Spin, "spin", callback_group=self.basic_nav_callback_group
        )
        self.follow_waypoints_client = PatchRclpyIssue1123(
            self,
            FollowWaypoints,
            "follow_waypoints",
            callback_group=self.basic_nav_callback_group,
        )

        ###########################################
        ### END NAV2 BASIC NAVIGATOR BASED CODE ###
        ###########################################

        self.get_logger().info("Autonomy task executor node initialized")

    #######################################
    ### NAV2 BASIC NAVIGATOR BASED CODE ###
    #######################################

    async def followGpsWaypoints(self, gps_poses):
        """
        Function to follow a set of GPS waypoints, based on the nav2_simple_commander code
        NOTE: Call this with the asyncio.run() function

        IMPORTANT! In ROS2 Humble the Nav2 GPS waypoint follower is not avaliable.
        I've implemented a patch to use robot_localization to convert to poses Nav2 can go to.
        For versions newer than Humble you should just be able to use that server though.

        https://github.com/ros-navigation/navigation2_tutorials/issues/77#issuecomment-1856414168
        """

        converted_poses = []
        for wp in gps_poses:
            pose = PoseStamped()
            pose.header.frame_id = "utm"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = utm.from_latlon(
                wp.position.latitude, wp.position.longitude
            )[0]
            pose.pose.position.y = utm.from_latlon(
                wp.position.latitude, wp.position.longitude
            )[1]
            pose.pose.orientation = wp.orientation

            converted_poses.append(pose)

        self.get_logger().info(
            f"Converted {len(gps_poses)} GPS waypoints to poses for Nav2"
        )
        await self.followWaypoints(converted_poses)

        # self.debug("Waiting for 'FollowGPSWaypoints' action server")
        # while not self.follow_gps_waypoints_client.wait_for_server(timeout_sec=1.0):
        #     self.info("'FollowGPSWaypoints' action server not available, waiting...")

        # goal_msg = FollowGPSWaypoints.Goal()
        # goal_msg.gps_poses = gps_poses

        # self.info(f"Following {len(goal_msg.gps_poses)} gps goals....")
        # send_goal_future = self.follow_gps_waypoints_client.send_goal_async(
        #     goal_msg, self._feedbackCallback
        # )
        # await send_goal_future  # fix for iron/humble threading bug
        # self.goal_handle = send_goal_future.result()

        # if not self.goal_handle.accepted:
        #     self.error("FollowGPSWaypoints request was rejected!")
        #     return False

        # self.result_future = self.goal_handle.get_result_async()
        # return True

    async def followWaypoints(self, poses):
        """
        Function to follow a set of GPS waypoints, based on the nav2_simple_commander code
        NOTE: Call this with the asyncio.run() function

        https://github.com/ros-navigation/navigation2_tutorials/issues/77#issuecomment-1856414168
        """

        self.debug("Waiting for 'FollowWaypoints' action server")
        while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowWaypoints' action server not available, waiting...")

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.info(f"Following {len(goal_msg.poses)} goals....")
        send_goal_future = self.follow_waypoints_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        await send_goal_future  # fix for iron/humble threading bug
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(f"Following {len(poses)} waypoints request was rejected!")
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    async def spin(
        self, spin_dist=1.57, time_allowance=10, disable_collision_checks=False
    ):
        """
        Function to spin in place, based on the nav2_simple_commander code
        NOTE: Call this with the asyncio.run() function
        """

        self.debug("Waiting for 'Spin' action server")
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist
        goal_msg.time_allowance = Duration(sec=time_allowance)
        # disable_collision_checks isn't in the iron/humble release of Nav2
        # goal_msg.disable_collision_checks = disable_collision_checks

        self.info(f"Spinning to angle {goal_msg.target_yaw}....")
        send_goal_future = self.spin_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        await send_goal_future  # fix for iron/humble threading bug
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error("Spin request was rejected!")
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    async def cancelTask(self):
        """
        Cancel pending task request of any type, based on the nav2_simple_commander code
        NOTE: Call this with the asyncio.run() function
        """

        self.info("Canceling current task.")
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            await future  # fix for iron/humble threading bug
        time.sleep(0.5)  # fix for bug, give time to cancel
        return

    async def isTaskComplete(self):
        """
        Check if the task request of any type is complete yet, based on the nav2_simple_commander code
        NOTE: Call this with the asyncio.run() function
        """

        if not self.result_future:
            # task was cancelled or completed
            return True

        # Fix for iron/humble threading bug (with timeout)
        # https://docs.python.org/3/library/asyncio-task.html#asyncio.wait_for
        try:
            await asyncio.wait_for(self.isTaskCompleteHelper(), timeout=0.1)
        except asyncio.TimeoutError:
            self.debug("Timed out waiting for async future to complete")

        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.error(f"Task with failed with status code: {self.status}")
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug("Task succeeded!")
        return True

    async def isTaskCompleteHelper(self):
        """
        Helper function for async 'wait_for' wrapping
        """

        await self.result_future

    def getFeedback(self):
        """
        Get the pending action feedback message, based on the nav2_simple_commander code
        """

        return self.feedback

    def getResult(self):
        """
        Get the pending action result message, based on the nav2_simple_commander code
        """

        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def waitUntilNav2Active(self, navigator="bt_navigator", localizer="amcl"):
        """
        Block until the full navigation system is up and running, based on the nav2_simple_commander code
        """

        if localizer != "robot_localization":  # non-lifecycle node
            asyncio.run(self._waitForNodeToActivate(localizer))
        if localizer == "amcl":
            self._waitForInitialPose()
        asyncio.run(self._waitForNodeToActivate(navigator))
        self.info("Nav2 is ready for use!")
        return

    async def _waitForNodeToActivate(self, node_name):
        """
        Waits for the node within the tester namespace to become active, based on the nav2_simple_commander code
        NOTE: Call this with the asyncio.run() function
        """

        self.debug(f"Waiting for {node_name} to become active..")
        node_service = f"{node_name}/get_state"
        state_client = self.create_client(
            GetState, node_service, callback_group=self.basic_nav_callback_group
        )
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f"{node_service} service not available, waiting...")

        req = GetState.Request()
        state = "unknown"
        while state != "active":
            self.info(f"Waiting for {node_name} to become active...")
            self.debug(f"Getting {node_name} state...")
            future = state_client.call_async(req)
            await future  # fix for iron/humble threading bug
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f"Result of get_state: {state}")
            time.sleep(2)
        return

    def _feedbackCallback(self, msg):
        self.debug("Received action feedback message")
        self.feedback = msg.feedback
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return

    ###########################################
    ### END NAV2 BASIC NAVIGATOR BASED CODE ###
    ###########################################

    #######################
    ### ROS 2 CALLBACKS ###
    #######################

    def cancel_callback(self, goal_handle):
        """
        Callback function for the action server cancel request
        """

        self.cancel_flag = True
        return CancelResponse.ACCEPT

    async def async_service_call(self, client, request):
        """
        Fix for iron/humble threading bug - https://github.com/ros2/rclpy/issues/1337
        NOTE: Call this with the asyncio.run() function (and all other async functions)
        """

        future = client.call_async(request)
        await future

    def action_server_callback(self, goal_handle):
        """
        Callback function for the action server
        """

        self.task_goal_handle = goal_handle
        self.cancel_flag = False
        result = AutonomyTask.Result()

        # Get the task legs from the goal
        self.legs = goal_handle.request.legs
        if not self.legs:
            self.task_fatal("No task legs provided")
            result.msg = "Well that was less-than-interstellar of you"
            self.task_goal_handle.abort()
            return result

        # Trigger the autonomy state
        asyncio.run(self.async_service_call(self.auto_client, self.auto_request))

        try:
            self.run_state_machine()
            result.msg = "One small step for a rover, one giant leap for roverkind"
            self.task_goal_handle.succeed()
        except Exception as e:  # catch exceptions to ensure we return to teleop state
            self.task_fatal(str(e))
            result.msg = "It was the aliens, I'm telling you"
            self.task_goal_handle.abort()

        # Trigger the teleop state
        asyncio.run(self.async_service_call(self.teleop_client, self.teleop_request))

        return result

    def gps_callback(self, msg):
        """
        Callback function for the GPS subscriber
        """

        # Set zone and hemisphere for UTM conversions
        if self.zone is None or self.hemisphere is None:
            self.zone = utm.from_latlon(msg.latitude, msg.longitude)[2]
            self.hemisphere = utm.from_latlon(msg.latitude, msg.longitude)[3]

        self.filtered_gps = latLonYaw2Geopose(msg.latitude, msg.longitude)

    def pose_to_geopose(self, pose, frame_id, stamp):
        """
        Convert a pose to a geopose we can navigate to
        """

        # Look up and use the transform to convert the pose to UTM
        try:
            tf = self.tf_buffer.lookup_transform("utm", frame_id, stamp)
            utm_pose = tf2_geometry_msgs.do_transform_pose(pose, tf)
        except Exception as e:
            self.get_logger().warn(f"Could not transform pose: {e}")
            return False

        # Check to make sure we've had at least one GPS fix
        if self.filtered_gps is None:
            self.get_logger().error("No filtered GPS fix available for UTM conversion")
            return False

        # Given UTM pose, convert to GPS
        lat, lon = utm.to_latlon(
            utm_pose.position.x,
            utm_pose.position.y,
            self.zone,
            self.hemisphere,
        )

        return latLonYaw2Geopose(lat, lon)

    def aruco_callback(self, msg):
        """
        Callback function for the aruco pose subscriber
        """

        if self.leg is None:
            return

        if self.leg.type == "aruco":
            for marker in msg.markers:
                # Are we looking for this marker right now?
                if marker.marker_id == self.leg.tag_id:

                    self.get_logger().info(f"Found aruco tag {marker.marker_id}")

                    # Convert the pose to a GeoPose
                    pose = self.pose_to_geopose(
                        marker.pose, msg.header.frame_id, msg.header.stamp
                    )

                    # If it was successful, store it
                    if pose:
                        self.found_poses[self.leg.name] = pose

    def obj_callback(self, msg):
        """
        Callback function for the object pose subscriber
        """

        if self.leg is None:
            return

        if self.leg.type == "obj":
            for obj in msg.objects:
                # Are we looking for this object right now?
                if obj.label == self.obj_to_label[self.leg.object]:

                    self.get_logger().info(f"Found object {obj.label}")

                    # Convert to a Pose() message
                    pose = Pose()
                    pose.position.x = float(obj.position[0])
                    pose.position.y = float(obj.position[1])
                    pose.position.z = float(obj.position[2])
                    pose.orientation.x = 0.0
                    pose.orientation.y = 0.0
                    pose.orientation.z = 0.0
                    pose.orientation.w = 1.0

                    # Convert the pose to a GeoPose
                    pose = self.pose_to_geopose(
                        pose, msg.header.frame_id, msg.header.stamp
                    )

                    # If it was successful, store it
                    if pose:
                        self.found_poses[self.leg.name] = pose

    ###########################
    ### END ROS 2 CALLBACKS ###
    ###########################

    ###############################
    ### TASK FEEDBACK FUNCTIONS ###
    ###############################

    def task_info(self, string):
        """
        Function to write info back to the AutonomyTask action client
        """

        name = "task" if self.leg is None else self.leg.name
        self.get_logger().info("[" + name + "] " + string)
        task_feedback = AutonomyTask.Feedback()
        task_feedback.status = "[" + name + "] " + string
        self.task_goal_handle.publish_feedback(task_feedback)

    def task_warn(self, string):
        """
        Function to write warnings back to the AutonomyTask action client
        """

        name = "task" if self.leg is None else self.leg.name
        self.get_logger().warn("[" + name + "] " + string)
        task_feedback = AutonomyTask.Feedback()
        task_feedback.status = "[WARN] [" + name + "] " + string
        self.task_goal_handle.publish_feedback(task_feedback)

    def task_error(self, string):
        """
        Function to write errors back to the AutonomyTask action client
        """

        name = "task" if self.leg is None else self.leg.name
        self.get_logger().error("[" + name + "] " + string)
        task_feedback = AutonomyTask.Feedback()
        task_feedback.status = "[ERROR] [" + name + "] " + string
        self.task_goal_handle.publish_feedback(task_feedback)

    def task_fatal(self, string):
        """
        Function to write fatal errors back to the AutonomyTask action client
        """

        name = "task" if self.leg is None else self.leg.name
        self.get_logger().fatal("[" + name + "] " + string)
        task_feedback = AutonomyTask.Feedback()
        task_feedback.status = "[FATAL] [" + name + "] " + string
        self.task_goal_handle.publish_feedback(task_feedback)

    def task_success(self, string):
        """
        Function to write success back to the AutonomyTask action client
        """

        name = "task" if self.leg is None else self.leg.name
        self.get_logger().info("[" + name + "] " + string)
        task_feedback = AutonomyTask.Feedback()
        task_feedback.status = "[SUCCESS] [" + name + "] " + string
        self.task_goal_handle.publish_feedback(task_feedback)

    ###################################
    ### END TASK FEEDBACK FUNCTIONS ###
    ###################################

    ########################
    ### HELPER FUNCTIONS ###
    ########################

    def navigate_helper(self, dest_wp, hex_mode=False, found_mode=False):
        """
        Helper waypoint navigation function that integrates with Nav2
        """

        # 1. Generate a path to the destination waypoint
        # Determine the best order for the legs
        try:
            planner_func = globals()[self.path_planner]
            if callable(planner_func):
                path = planner_func(self.filtered_gps, dest_wp, self.waypoint_distance)
            else:
                raise Exception(
                    "Path planner " + str(self.path_planner) + " is not callable"
                )
        except KeyError:
            raise Exception("Path planner " + str(self.path_planner) + " not found")

        # 2. Publish the GPS positions to mapviz
        for wp in path:
            navsat_fix = NavSatFix()
            navsat_fix.header.frame_id = "map"
            navsat_fix.header.stamp = self.get_clock().now().to_msg()
            navsat_fix.latitude = wp.position.latitude
            navsat_fix.longitude = wp.position.longitude

            # 2.1 Publish to different topics based on type
            if wp != dest_wp:
                self.mapviz_inter_publisher.publish(navsat_fix)
            else:
                self.mapviz_goal_publisher.publish(navsat_fix)

        # 3. Follow the determined path
        asyncio.run(self.followGpsWaypoints(path))
        start_time = self.get_clock().now().to_msg()
        while not asyncio.run(self.isTaskComplete()):
            time.sleep(0.1)

            # 3.1 Check if the goal has been canceled
            if self.task_goal_handle.is_cancel_requested:
                asyncio.run(self.cancelTask())
                raise Exception("Task execution canceled by action client")

            # 3.2 Check if we've spent too long on this waypoint and should move on
            if not hex_mode and not found_mode:  # we're navigating to a GPS waypoint
                if (
                    self.get_clock().now().to_msg().sec - start_time.sec
                    > self.gps_nav_timeout
                ):
                    self.task_error("GPS navigation timed out")
                    asyncio.run(self.cancelTask())
                    return Result.FAILED
            elif hex_mode:  # we're navigating to a hex waypoint
                if (
                    self.get_clock().now().to_msg().sec - start_time.sec
                    > self.hex_nav_timeout
                ):
                    self.task_error("Hex navigation timed out")
                    asyncio.run(self.cancelTask())
                    return Result.FAILED

            # 3.3 Check for the object or aruco tag while navigating
            if found_mode:  # we're navigating to a found object
                # Check if we get a better object or tag location as we get closer
                if (
                    latLon2Meters(
                        self.found_poses[self.leg.name].position.latitude,
                        self.found_poses[self.leg.name].position.longitude,
                        dest_wp.position.latitude,
                        dest_wp.position.longitude,
                    )
                    > self.update_threshold
                ):
                    asyncio.run(self.cancelTask())
                    return Result.FOUND
            else:  # we're navigating to a GPS or hex waypoint
                # Check for the aruco tag or object while navigating
                if self.found_helper():
                    asyncio.run(self.cancelTask())
                    return Result.FOUND

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            return Result.SUCCEEDED
        else:
            return Result.FAILED

    def spin_helper(self):
        """
        Helper spin function that integrates with Nav2
        """

        # 1. Calculate the angle
        adj_spin_distance = 6.28 / self.spin_stops

        # 2. Spin to the calculated angle
        asyncio.run(self.spin(spin_dist=adj_spin_distance))
        while not asyncio.run(self.isTaskComplete()):
            time.sleep(0.1)

            # 2.1 Check if the goal has been canceled
            if self.task_goal_handle.is_cancel_requested:
                asyncio.run(self.cancelTask())
                raise Exception("Task execution canceled by action client")

            # 2.2 Check for the aruco tag or object while spinning
            if self.found_helper():
                asyncio.run(self.cancelTask())
                return Result.FOUND

        # 3. Wait a designated amount of time to get a good picture
        time.sleep(self.spin_wait_time)

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            return Result.SUCCEEDED
        elif result == TaskResult.CANCELED or result == TaskResult.FAILED:
            return Result.FAILED

    def found_helper(self):
        """
        Helper function to check for aruco tags and objects
        """

        # 1. Check if we have a pose for the object or aruco tag
        if self.leg.type == "aruco" or self.leg.type == "obj":
            try:
                if self.found_poses[self.leg.name]:
                    return True
            except KeyError:  # if the leg name is not in the found_poses dict
                return False
        return False

    ############################
    ### END HELPER FUNCTIONS ###
    ############################

    #####################
    ### STATE MACHINE ###
    #####################

    def run_state_machine(self):
        """
        Function to run the state machine
        """

        self.complete_flag = False
        self.state = State.INIT

        while not self.complete_flag:
            self.get_logger().info("Transitioned to state: " + str(self.state))
            match self.state:
                case State.INIT:
                    self.handle_init()
                case State.NEXT_LEG:
                    self.handle_next_leg()
                case State.GPS_NAV:
                    self.handle_gps_nav()
                case State.SPIN_SEARCH:
                    self.handle_spin_search()
                case State.NEXT_HEX:
                    self.handle_next_hex()
                case State.HEX_NAV:
                    self.handle_hex_nav()
                case State.FOUND_NAV:
                    self.handle_found_nav()
                case State.SIGNAL_SUCCESS:
                    self.handle_signal_success()
                case _:
                    raise Exception("Invalid state: " + str(self.state))

    def handle_init(self):
        """
        Function to handle the initialization state
        """

        self.task_info("Autonomy task started")

        # Initialize variables
        self.found_poses = {}
        self.leg_cntr = 0
        self.hex_cntr = 0
        self.coord = None

        # Check for the first GPS fix
        while self.filtered_gps is None:
            time.sleep(1)
            self.task_warn("Waiting on a GPS fix...")

            # Check if the goal has been canceled
            if self.task_goal_handle.is_cancel_requested:
                asyncio.run(self.cancelTask())
                raise Exception("Task execution canceled by action client")

        # Report which order and path planners are selected
        self.task_info("Order planner: " + self.order_planner)
        self.task_info("Path planner: " + self.path_planner)

        # Determine the best order for the legs
        try:
            planner_func = globals()[self.order_planner]
            if callable(planner_func):
                self.legs = planner_func(self.legs, self.filtered_gps)
            else:
                raise Exception(
                    "Order planner " + str(self.order_planner) + " is not callable"
                )
        except KeyError:
            raise Exception("Order planner " + str(self.order_planner) + " not found")

        order = [leg.name for leg in self.legs]
        self.task_info("Determined best leg order: " + str(order))

        # Make sure the navigation stack is up and running
        self.waitUntilNav2Active(localizer="robot_localization")

        self.state = State.NEXT_LEG

    def handle_next_leg(self):
        """
        Function to handle the next leg state
        """

        # Have we completed all the legs?
        if self.leg_cntr >= len(self.legs):
            self.leg = None
            self.task_info("Autonomy task completed")
            self.complete_flag = True
            return

        self.leg = self.legs[self.leg_cntr]
        self.leg_cntr += 1

        self.hex_cntr = 0  # reset the hex counter

        # Enable the object detection
        if self.leg.type == "obj":
            self.task_info("Enabling object detection")
            self.obj_request.data = True
            asyncio.run(self.async_service_call(self.obj_client, self.obj_request))

        self.state = State.GPS_NAV

    def handle_gps_nav(self):
        """
        Function to handle the GPS navigation state
        """

        self.task_info("Starting GPS navigation")

        leg_wp = latLonYaw2Geopose(self.leg.latitude, self.leg.longitude)
        nav_result = self.navigate_helper(leg_wp)

        if nav_result == Result.SUCCEEDED:

            if self.leg.type == "gps":
                self.task_success("Navigated to GPS waypoint")
                self.state = State.SIGNAL_SUCCESS
            else:
                self.task_info("Navigated to GPS waypoint")
                self.state = State.SPIN_SEARCH

        elif nav_result == Result.FAILED:
            self.task_error("Failed to navigate to GPS waypoint")

            # Disable the object detection
            if self.leg.type == "obj":
                self.task_info("Disabling object detection")
                self.obj_request.data = False
                asyncio.run(self.async_service_call(self.obj_client, self.obj_request))

            self.state = State.NEXT_LEG
        elif nav_result == Result.FOUND:
            self.task_info("Found the object/tag")
            self.state = State.FOUND_NAV

    def handle_spin_search(self):
        """
        Function to handle the spin search state
        """

        self.task_info("Starting spin search")

        success_cnt = 0
        failure_cnt = 0
        found_flag = False

        for i in range(
            self.spin_stops - 1
        ):  # don't want to spin back to where we started

            spin_result = self.spin_helper()

            if spin_result == Result.SUCCEEDED:
                success_cnt += 1
            elif spin_result == Result.FAILED:
                failure_cnt += 1
            elif spin_result == Result.FOUND:
                found_flag = True
                break

        if not found_flag:
            self.task_info(
                "Spin search completed ("
                + str(success_cnt)
                + "/"
                + str(failure_cnt + success_cnt)
                + ")"
            )
            self.state = State.NEXT_HEX
        else:
            self.task_info("Found the object/tag")
            self.state = State.FOUND_NAV

    def handle_next_hex(self):
        """
        Function to handle the next hex state
        """

        # Have we completed all the hex points?
        if self.hex_cntr >= len(self.hex_coord):
            self.task_error("Failed to find the object/tag")
            self.state = State.NEXT_LEG
            return

        self.coord = self.hex_coord[self.hex_cntr]
        self.hex_cntr += 1

        self.state = State.HEX_NAV

    def handle_hex_nav(self):
        """
        Function to handle the hex navigation state
        """

        self.task_info("Starting hex " + str(self.hex_cntr) + " navigation")

        hex_lat, hex_lon = meters2LatLon(
            self.leg.latitude,
            self.leg.longitude,
            self.coord[0],
            self.coord[1],
        )
        hex_wp = latLonYaw2Geopose(hex_lat, hex_lon)
        nav_result = self.navigate_helper(hex_wp, hex_mode=True)

        if nav_result == Result.SUCCEEDED:
            self.task_info("Navigated to hex " + str(self.hex_cntr))
            self.state = State.SPIN_SEARCH
        elif nav_result == Result.FAILED:
            self.task_error("Failed to navigate to hex " + str(self.hex_cntr))
            self.state = State.NEXT_HEX
        elif nav_result == Result.FOUND:
            self.task_info("Found the object/tag")
            self.state = State.FOUND_NAV

    def handle_found_nav(self):
        """
        Function to handle the found navigation state
        """

        self.task_info("Starting object/tag navigation")

        found_wp = latLonYaw2Geopose(
            self.found_poses[self.leg.name].position.latitude,
            self.found_poses[self.leg.name].position.longitude,
        )
        nav_result = self.navigate_helper(found_wp, found_mode=True)

        if nav_result == Result.SUCCEEDED:
            self.task_success("Navigated to object/tag")

            # Disable the object detection
            if self.leg.type == "obj":
                self.task_info("Disabling object detection")
                self.obj_request.data = False
                asyncio.run(self.async_service_call(self.obj_client, self.obj_request))

            self.state = State.SIGNAL_SUCCESS
        elif nav_result == Result.FAILED:
            self.task_error("Failed to navigate to object/tag")

            # Disable the object detection
            if self.leg.type == "obj":
                self.task_info("Disabling object detection")
                self.obj_request.data = False
                asyncio.run(self.async_service_call(self.obj_client, self.obj_request))

            self.state = State.NEXT_LEG
        elif nav_result == Result.FOUND:
            self.task_info("Updated location for object/tag")
            self.state = State.FOUND_NAV

    def handle_signal_success(self):
        """
        Function to handle the signal success state
        """

        self.task_info("Flashing LED to indicate arrival")

        # Trigger the arrival state
        asyncio.run(self.async_service_call(self.arrival_client, self.arrival_request))

        time.sleep(self.wait_time)

        # Trigger the autonomy state
        asyncio.run(self.async_service_call(self.auto_client, self.auto_request))

        self.state = State.NEXT_LEG

    #########################
    ### END STATE MACHINE ###
    #########################


def main(args=None):
    rclpy.init(args=args)

    state_machine = StateMachine()
    # Create a multi-threaded node executor for callback-in-callback threading
    executor = MultiThreadedExecutor()
    executor.add_node(state_machine)

    executor.spin()


if __name__ == "__main__":
    main()
