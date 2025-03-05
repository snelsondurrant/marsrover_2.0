# Created by Nelson Durrant, Feb 2025
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from nav2_msgs.action import FollowGPSWaypoints, Spin
from rover_interfaces.action import RunBT
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger
from lifecycle_msgs.srv import GetState
from aruco_opencv_msgs.msg import ArucoDetection
from vision_msgs.msg import Detection3DArray
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import asyncio
import tf2_geometry_msgs
import tf2_ros
import utm
import yaml
import time

from nav2_autonomy.utils.gps_utils import latLonYaw2Geopose
from nav2_autonomy.utils.plan_utils import (
    basicPathPlanner,  # plan a straight line between two GPS coordinates
    bruteOrderPlanner,  # use brute force to find the best order of legs
    greedyOrderPlanner,  # use a greedy algorithm to find the best order of legs
    noOrderPlanner,  # don't reorder the legs
)
from nav2_autonomy.utils.terrain_utils import (
    terrainPathPlanner,
    terrainOrderPlanner,
)

#####################################################
### Select the path and leg order planners to use ###
#####################################################

globals()["__path_planner__"] = basicPathPlanner
globals()["__order_planner__"] = greedyOrderPlanner


class YamlParser:
    """
    Parse a set of GPS waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, "r") as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of GPS waypoint yaml objects from the yaml file
        """
        gps_wps = []
        for wp in self.wps_dict["waypoints"]:
            gps_wps.append(wp)
        return gps_wps


class BehaviorTree(Node):
    """
    Class for running the autonomy task behavior tree using Nav2

    Note: This is a pretty complex node. It's a hacked-together combination of the BasicNavigator
    class and our own custom behavior tree with a lot of multi-threading. It's easiest to think of
    as three sections with their own seperate threads: the behavior tree, the Nav2 BasicNavigator,
    and the ROS 2 callbacks.

    :author: Nelson Durrant
    :date: Mar 2025

    Subscribers:
    - gps/filtered (sensor_msgs/NavSatFix)
    - aruco_detections (aruco_opencv_msgs/ArucoDetection)
    - zed/detections (vision_msgs/Detection3DArray)
    Publishers:
    - mapviz/goal (sensor_msgs/NavSatFix)
    - mapviz/inter (sensor_msgs/NavSatFix)
    Clients:
    - trigger_teleop (std_srvs/Trigger)
    - trigger_auto (std_srvs/Trigger)
    - trigger_arrival (std_srvs/Trigger)
    Action Clients:
    - follow_gps_waypoints (nav2_msgs/FollowGPSWaypoints)
    - spin (nav2_msgs/Spin)
    - {node_name}/get_state (lifecycle_msgs/GetState) (temporary)
    Action Servers:
    - run_bt (rover_interfaces/RunBT)
    *And a tf2 buffer and listener for pose to GPS transforms
    """

    def __init__(self):

        super().__init__("behavior_tree")
        # self.navigator = BasicNavigator() # Don't uncomment this line
        # IMPORTANT! Simply using the BasicNavigator class causes A LOT of threading issues.
        # We've hacked the relevant functions from the BasicNavigator class into this class as a fix.
        # https://github.com/ros-navigation/navigation2/tree/main/nav2_simple_commander

        # Parse the waypoint file
        self.declare_parameter("wps_file_path", "")
        if not self.get_parameter("wps_file_path").value:
            self.get_logger().fatal("No waypoint file path provided")
            rclpy.shutdown()

        wps_file_path = self.get_parameter("wps_file_path").value
        self.wp_parser = YamlParser(wps_file_path)
        self.wps = self.wp_parser.get_wps()

        # Task constants
        self.gps_legs = ["gps1", "gps2"]
        self.aruco_legs = ["aruco1", "aruco2", "aruco3"]
        self.obj_legs = ["mallet", "bottle"]

        # Initialize variables
        self.legs = []
        self.leg = ""
        self.filtered_gps = None

        # Pose dictionaries
        self.tags = {"aruco1": 1, "aruco2": 2, "aruco3": 3}
        self.aruco_poses = {"aruco1": None, "aruco2": None, "aruco3": None}
        self.obj_poses = {"mallet": None, "bottle": None}

        # UTM zone and hemisphere (will set on first gps fix)
        self.zone = None
        self.hemisphere = None

        # Tunable values
        self.wait_time = 10  # Time to wait for a task to complete
        self.update_threshold = 0.00001  # Threshold for updating tag and item locations

        # Hex pattern for searching
        self.hex_coord = [
            (1.0, 0.0),
            (0.5, 0.866),
            (-0.5, 0.866),
            (-1.0, 0.0),
            (-0.5, -0.866),
            (0.5, -0.866),
        ]
        self.hex_scalar = 0.0001

        ################################
        ### ROS 2 OBJECT DEFINITIONS ###
        ################################

        # Set up a Tf2 buffer for pose to GPS transforms (aruco, object)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Callback groups (for threading)
        bg_callback_group = MutuallyExclusiveCallbackGroup()
        fg_callback_group = MutuallyExclusiveCallbackGroup()

        # GPS location (filtered) subscriber
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            "gps/filtered",
            self.gps_callback,
            10,
            callback_group=bg_callback_group,
        )
        self.gps_subscriber  # prevent unused variable warning

        # Aruco pose subscriber
        self.aruco_subscriber = self.create_subscription(
            ArucoDetection,
            "aruco_detections",
            self.aruco_callback,
            10,
            callback_group=bg_callback_group,
        )
        self.aruco_subscriber  # prevent unused variable warning

        # Object pose subscriber
        self.obj_subscriber = self.create_subscription(
            Detection3DArray,
            "zed/detections",
            self.obj_callback,
            10,
            callback_group=bg_callback_group,
        )
        self.obj_subscriber  # prevent unused variable warning

        # Mapviz publishers (to show the goals in mapviz)
        self.mapviz_goal_publisher = self.create_publisher(NavSatFix, "mapviz/goal", 10)
        self.mapviz_inter_publisher = self.create_publisher(
            NavSatFix, "mapviz/inter", 10
        )

        # Client to trigger teleop state
        self.teleop_client = self.create_client(
            Trigger, "trigger_teleop", callback_group=bg_callback_group
        )
        while not self.teleop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Teleop trigger service not available, waiting again..."
            )
        self.teleop_request = Trigger.Request()

        # Client to trigger autonomy state
        self.nav_client = self.create_client(
            Trigger, "trigger_auto", callback_group=bg_callback_group
        )
        while not self.nav_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Autonomy trigger service not available, waiting again..."
            )
        self.nav_request = Trigger.Request()

        # Client to trigger arrival state
        self.arrival_client = self.create_client(
            Trigger, "trigger_arrival", callback_group=bg_callback_group
        )
        while not self.arrival_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Arrival trigger service not available, waiting again..."
            )
        self.arrival_request = Trigger.Request()

        # Action server to run the behavior tree
        self.action_server = ActionServer(
            self,
            RunBT,
            "run_bt",
            self.action_server_callback,
            callback_group=fg_callback_group,
        )

        ####################################
        ### END ROS 2 OBJECT DEFINITIONS ###
        ####################################

        #######################################
        ### NAV2 BASIC NAVIGATOR BASED CODE ###
        #######################################

        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        self.cmd_callback_group = MutuallyExclusiveCallbackGroup()
        self.follow_gps_waypoints_client = ActionClient(
            self,
            FollowGPSWaypoints,
            "follow_gps_waypoints",
            callback_group=self.cmd_callback_group,
        )
        self.spin_client = ActionClient(
            self, Spin, "spin", callback_group=self.cmd_callback_group
        )

        ###########################################
        ### END NAV2 BASIC NAVIGATOR BASED CODE ###
        ###########################################

        self.get_logger().info("State machine node initialized")

    #######################################
    ### NAV2 BASIC NAVIGATOR BASED CODE ###
    #######################################

    async def followGpsWaypoints(self, gps_poses):
        """
        Function to follow a set of GPS waypoints, based on the nav2_simple_commander code
        NOTE: Call this with the asyncio.run() function
        """

        self.debug("Waiting for 'FollowWaypoints' action server")
        while not self.follow_gps_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowWaypoints' action server not available, waiting...")

        goal_msg = FollowGPSWaypoints.Goal()
        goal_msg.gps_poses = gps_poses

        self.info(f"Following {len(goal_msg.gps_poses)} gps goals....")
        send_goal_future = self.follow_gps_waypoints_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        await send_goal_future  # fix for iron threading bug
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
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
        # goal_msg.disable_collision_checks = disable_collision_checks

        self.info(f"Spinning to angle {goal_msg.target_yaw}....")
        send_goal_future = self.spin_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        await send_goal_future  # fix for iron threading bug
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
            await future  # fix for iron threading bug
        return

    async def isTaskComplete(self):
        """
        Check if the task request of any type is complete yet, based on the nav2_simple_commander code
        NOTE: Call this with the asyncio.run() function
        """

        if not self.result_future:
            # task was cancelled or completed
            return True
        
        # Fix for iron threading bug (with timeout)
        # https://docs.python.org/3/library/asyncio-task.html#asyncio.wait_for
        try:
            await asyncio.wait_for(self.isTaskCompleteHelper(), timeout=0.1)
        except asyncio.TimeoutError:
            self.debug('Timed out waiting for async future to complete')

        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f"Task with failed with status code: {self.status}")
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
        # if localizer == 'amcl':
        #     self._waitForInitialPose()
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
            GetState, node_service, callback_group=self.cmd_callback_group
        )
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f"{node_service} service not available, waiting...")

        req = GetState.Request()
        state = "unknown"
        while state != "active":
            self.debug(f"Getting {node_name} state...")
            future = state_client.call_async(req)
            await future  # fix for iron threading bug
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

    async def async_service_call(self, client, request):
        """
        Fix for iron threading bug - https://github.com/ros2/rclpy/issues/1337
        NOTE: Call this with the asyncio.run() function (and all other async functions)
        """

        future = client.call_async(request)
        await future

    def action_server_callback(self, goal_handle):
        """
        Callback function for the action server
        """

        self.sm_goal_handle = goal_handle

        # Get the task legs from the goal
        self.legs = goal_handle.request.legs
        if not self.legs:
            self.bt_fatal("No task legs provided")
            result = RunBT.Result()
            result.congrats = "It was the aliens, I'm telling you"
            self.sm_goal_handle.abort()
            return result

        # Trigger the autonomy state
        asyncio.run(self.async_service_call(self.nav_client, self.nav_request))

        try:
            self.leg = "start"
            self.run_behavior_tree()
        except Exception as e:
            self.bt_fatal(str(e))
            result = RunBT.Result()
            result.congrats = "It was the aliens, I'm telling you"
            self.sm_goal_handle.abort()
            return result

        # Trigger the teleop state
        asyncio.run(self.async_service_call(self.teleop_client, self.teleop_request))

        result = RunBT.Result()
        result.congrats = "One small step for a rover, one giant leap for roverkind"
        self.sm_goal_handle.succeed()
        return result

    def gps_callback(self, msg):
        """
        Callback function for the GPS subscriber
        """

        # Set zone and hemisphere for UTM conversions
        self.zone = utm.from_latlon(msg.latitude, msg.longitude)[2]
        self.hemisphere = "N" if msg.latitude > 0 else "S"

        self.filtered_gps = latLonYaw2Geopose(msg.latitude, msg.longitude)

    def aruco_callback(self, msg):
        """
        Callback function for the aruco pose subscriber
        """

        for marker in msg.markers:
            # Are we looking for this marker?
            if marker.marker_id in self.tags.values():

                self.get_logger().info(f"Found aruco tag {marker.marker_id}")

                # Look up and use the transform to convert the pose to UTM
                try:
                    tf = self.tf_buffer.lookup_transform(
                        "utm", msg.header.frame_id, msg.header.stamp
                    )
                    utm_pose = tf2_geometry_msgs.do_transform_pose(marker.pose, tf)
                except Exception as e:
                    self.get_logger().warn(f"Could not transform aruco pose: {e}")
                    return

                # Check to make sure we've had at least one GPS fix
                if self.filtered_gps is None:
                    self.get_logger().error(
                        "No filtered GPS fix available for UTM conversion"
                    )
                    return

                # Given UTM pose, convert to GPS
                lat, lon = utm.to_latlon(
                    utm_pose.position.x,
                    utm_pose.position.y,
                    self.zone,
                    self.hemisphere,
                )

                self.aruco_poses[self.leg] = latLonYaw2Geopose(lat, lon)

    def obj_callback(self, msg):
        """
        Callback function for the object detection subscriber
        """

        # Detection3DArray:
        #   std_msgs/Header header
        #   vision_msgs/Detection3D[] detections
        # Detection3D:
        #   vision_msgs/BoundingBox3D bounding_box
        #   vision_msgs/ObjectHypothesisWithPose[] results
        # ObjectHypothesisWithPose:
        #   vision_msgs/ObjectHypothesis hypothesis
        #   geometry_msgs/PoseStamped pose

        # TODO: Implement this function

        return

    ###########################
    ### END ROS 2 CALLBACKS ###
    ###########################

    ####################################
    ### BT ACTION FEEDBACK FUNCTIONS ###
    ####################################

    def bt_info(self, string):
        """
        Function to write info back to the RunBT action client
        """

        self.get_logger().info("[" + self.leg + "] " + string)
        sm_feedback = RunBT.Feedback()
        sm_feedback.status = "[" + self.leg + "] " + string
        self.sm_goal_handle.publish_feedback(sm_feedback)

    def bt_warn(self, string):
        """
        Function to write warnings back to the RunBT action client
        """

        self.get_logger().warn("[" + self.leg + "] " + string)
        sm_feedback = RunBT.Feedback()
        sm_feedback.status = "[WARN] [" + self.leg + "] " + string
        self.sm_goal_handle.publish_feedback(sm_feedback)

    def bt_error(self, string):
        """
        Function to write errors back to the RunBT action client
        """

        self.get_logger().error("[" + self.leg + "] " + string)
        sm_feedback = RunBT.Feedback()
        sm_feedback.status = "[ERROR] [" + self.leg + "] " + string
        self.sm_goal_handle.publish_feedback(sm_feedback)

    def bt_fatal(self, string):
        """
        Function to write fatal errors back to the RunBT action client
        """

        self.get_logger().fatal("[" + self.leg + "] " + string)
        sm_feedback = RunBT.Feedback()
        sm_feedback.status = "[FATAL] [" + self.leg + "] " + string
        self.sm_goal_handle.publish_feedback(sm_feedback)

    ########################################
    ### END BT ACTION FEEDBACK FUNCTIONS ###
    ########################################

    #####################
    ### BEHAVIOR TREE ###
    #####################

    def run_behavior_tree(self):
        """
        Function to run the competition behavior tree
        """

        self.bt_info("Behavior tree started")

        self.bt_info("Using order planner: " + globals()["__order_planner__"].__name__)
        self.bt_info("Using path planner: " + globals()["__path_planner__"].__name__)

        # Determine the best order for the legs
        self.legs = globals()["__order_planner__"](
            self.legs, self.wps, self.filtered_gps
        )

        self.bt_info("Determined best leg order: " + str(self.legs))

        self.waitUntilNav2Active(localizer="robot_localization")

        for leg in self.legs:
            self.exec_leg(leg)

        self.leg = "end"
        self.bt_info("Behavior tree completed")

    def exec_leg(self, leg):
        """
        Function to execute task legs
        """

        self.leg = leg

        # Is it a GPS leg?
        if self.leg in self.gps_legs:

            self.bt_info("Starting GPS leg")

            # Get this leg's GPS waypoint
            leg_wp = None
            for wp in self.wps:
                if wp["leg"] == self.leg:
                    leg_wp = latLonYaw2Geopose(wp["latitude"], wp["longitude"])

            if not leg_wp:
                self.bt_fatal("No GPS waypoint defined for leg")
                return False

            self.gps_nav(leg_wp)

            self.bt_info("SUCCESS! Navigated to GPS waypoint")

            # Trigger the arrival state
            asyncio.run(
                self.async_service_call(self.arrival_client, self.arrival_request)
            )

            time.sleep(self.wait_time)

            # Trigger the autonomy state
            asyncio.run(self.async_service_call(self.nav_client, self.nav_request))

        # Is it an aruco leg?
        elif self.leg in self.aruco_legs:

            self.bt_info("Starting aruco leg")

            # Get this leg's GPS waypoint
            leg_wp = None
            for wp in self.wps:
                if wp["leg"] == self.leg:
                    leg_wp = latLonYaw2Geopose(wp["latitude"], wp["longitude"])

            if not leg_wp:
                self.bt_fatal("No GPS waypoint defined for leg")
                return False

            # Navigate to the GPS waypoint
            self.gps_nav(leg_wp)

            # Look for the aruco tag
            aruco_loc = self.spin_search()  # Do a spin search
            if not aruco_loc:
                aruco_loc = self.hex_search()  # Do a hex search
            if not aruco_loc:
                self.bt_error("Could not find the aruco tag")
            else:
                self.bt_info("Found the aruco tag!")

                # Loop through with an updating GPS location
                finished = False
                while not finished:
                    finished = self.gps_nav(aruco_loc, " (aruco tag)", updating=True)
                    if not finished:
                        aruco_loc = self.aruco_check()

                self.bt_info("SUCCESS! Found and navigated to aruco tag")

                # Trigger the arrival state
                asyncio.run(
                    self.async_service_call(self.arrival_client, self.arrival_request)
                )

                time.sleep(self.wait_time)

                # Trigger the autonomy state
                asyncio.run(self.async_service_call(self.nav_client, self.nav_request))

        # Is it an object leg?
        elif leg in self.obj_legs:

            self.bt_info("Starting object leg")

            # Get this leg's GPS waypoint
            leg_wp = None
            for wp in self.wps:
                if wp["leg"] == self.leg:
                    leg_wp = latLonYaw2Geopose(wp["latitude"], wp["longitude"])

            if not leg_wp:
                self.bt_fatal("No GPS waypoint defined for leg")
                return False

            # Navigate to the GPS waypoint
            self.gps_nav(leg_wp)

            # Look for the object
            obj_loc = self.spin_search()  # Do a spin search
            if not obj_loc:
                obj_loc = self.hex_search()  # Do a hex search
            if not obj_loc:
                self.bt_error("Could not find the object")
            else:
                self.bt_info("Found the object!")

                # Loop through with an updating GPS location
                finished = False
                while not finished:
                    finished = self.gps_nav(obj_loc, " (object)", updating=True)
                    if not finished:
                        obj_loc = self.obj_check()

                self.bt_info("SUCCESS! Found and navigated to object")

                # Trigger the arrival state
                asyncio.run(
                    self.async_service_call(self.arrival_client, self.arrival_request)
                )

                time.sleep(self.wait_time)

                # Trigger the autonomy state
                asyncio.run(self.async_service_call(self.nav_client, self.nav_request))

        else:
            self.bt_fatal("Invalid leg type provided")
            return False

    def gps_nav(self, dest_wp, src_string="", updating=False):
        """
        Function to navigate through GPS waypoints
        """

        self.bt_info("Starting GPS navigation" + src_string)

        # Generate a path to the destination waypoint
        path = globals()["__path_planner__"](self.filtered_gps, dest_wp)

        # Publish the GPS positions to mapviz
        for wp in path:
            navsat_fix = NavSatFix()
            navsat_fix.header.frame_id = "map"
            navsat_fix.header.stamp = self.get_clock().now().to_msg()
            navsat_fix.latitude = wp.position.latitude
            navsat_fix.longitude = wp.position.longitude

            # Publish to different topics based on type
            if wp != dest_wp:
                self.mapviz_inter_publisher.publish(navsat_fix)
            else:
                self.mapviz_goal_publisher.publish(navsat_fix)

        asyncio.run(self.followGpsWaypoints(path))
        while not asyncio.run(self.isTaskComplete()):
            time.sleep(0.1)

            # See if we get a better pose from the aruco tag or object
            if updating:
                if self.leg in self.aruco_legs:
                    pose = self.aruco_check()
                elif self.leg in self.obj_legs:
                    pose = self.obj_check()

                # Check if its location has changed by a significant amount
                if (
                    abs(pose.position.latitude - dest_wp.position.latitude)
                    > self.update_threshold
                ) or (
                    abs(pose.position.longitude - dest_wp.position.longitude)
                    > self.update_threshold
                ):
                    self.bt_info("Improved GPS location found" + src_string)
                    asyncio.run(self.cancelTask())
                    return False # restart gps_nav with the new location

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.bt_info("GPS navigation completed" + src_string)
        elif result == TaskResult.CANCELED:
            self.bt_warn("GPS navigation canceled" + src_string)
        elif result == TaskResult.FAILED:
            self.bt_error("GPS navigation failed" + src_string)

        return True  # for updating mode logic

    def spin_search(self, src_string=""):
        """
        Function to perform a spin search
        """

        self.bt_info("Starting spin search" + src_string)

        asyncio.run(self.spin(spin_dist=3.14))
        while not asyncio.run(self.isTaskComplete()):

            if self.leg in self.aruco_legs:
                # Check for the aruco tag
                pose = self.aruco_check()
                if pose:
                    asyncio.run(self.cancelTask())
                    return pose

            elif self.leg in self.obj_legs:
                # Check for the object
                pose = self.obj_check()
                if pose:
                    asyncio.run(self.cancelTask())
                    return pose

            time.sleep(0.1)

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.bt_info("Spin search completed" + src_string)
        elif result == TaskResult.CANCELED:
            self.bt_warn("Spin search canceled" + src_string)
        elif result == TaskResult.FAILED:
            self.bt_error("Spin search failed" + src_string)
        return False

    def hex_search(self):
        """
        Function to search in a hex pattern
        """

        self.bt_info("Starting hex search")

        # Get the base waypoint
        for wp in self.wps:
            if wp["leg"] == self.leg:
                base_wp = latLonYaw2Geopose(wp["latitude"], wp["longitude"])

        # Generate a hex pattern from the base waypoint
        for i, coord in enumerate(self.hex_coord):
            hex_lat = base_wp.position.latitude + coord[0] * self.hex_scalar
            hex_lon = base_wp.position.longitude + coord[1] * self.hex_scalar
            hex_wp = latLonYaw2Geopose(hex_lat, hex_lon)

            self.gps_nav(hex_wp, " (hex " + str(i) + ")")
            pose = self.spin_search(" (hex " + str(i) + ")")  # Do a spin search
            # Did the last spin search find it?
            if pose:
                return pose

        self.bt_info("Hex search completed")
        return False

    def aruco_check(self):
        """
        Function to check for the aruco tag
        """

        if self.aruco_poses[self.leg]:
            return self.aruco_poses[self.leg]

        return False

    def obj_check(self):
        """
        Function to check for the object
        """

        if self.obj_poses[self.leg]:
            return self.obj_poses[self.leg]

        return False

    ###################################
    ### END BEHAVIOR TREE FUNCTIONS ###
    ###################################


def main(args=None):
    rclpy.init(args=args)

    behavior_tree = BehaviorTree()
    # Create a multi-threaded executor for Nav2 locking issues
    executor = MultiThreadedExecutor()
    executor.add_node(behavior_tree)

    executor.spin()


if __name__ == "__main__":
    main()
