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
from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose
from vision_msgs.msg import Detection3DArray, ObjectHypothesisWithPose
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import yaml
import time

from nav2_autonomy.utils.gps_utils import latLonYaw2Geopose
from nav2_autonomy.utils.plan_utils import basicPathPlanner


class YamlParser:
    """
    Parse a set of legs, aruco tags, and GPS waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, "r") as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_legs(self):
        """
        Get an array of leg strings from the yaml file
        """

        legs = []
        for leg in self.wps_dict["legs"]:
            legs.append(leg["leg"])
        return legs
    
    def get_tags(self):
        """
        Get an array of aruco tag yaml objects from the yaml file
        """
        aruco_tags = []
        for tag in self.wps_dict["aruco_tags"]:
            aruco_tags.append(tag)
        return aruco_tags

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
    :date: Feb 2025

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
    """

    def __init__(self):

        super().__init__("behavior_tree")
        # self.navigator = BasicNavigator() # Don't uncomment this line
        # IMPORTANT! Simply using the BasicNavigator class causes A LOT of threading issues
        # We've hacked the relevant functions from the BasicNavigator class into this class as a fix

        # Parse the waypoint file
        self.declare_parameter("wps_file_path", "")
        if not self.get_parameter("wps_file_path").value:
            self.get_logger().fatal("No waypoint file path provided")
            rclpy.shutdown()

        wps_file_path = self.get_parameter("wps_file_path").value
        self.wp_parser = YamlParser(wps_file_path)

        self.legs = self.wp_parser.get_legs() # array of strings
        self.tags = self.wp_parser.get_tags() # array of yaml
        self.wps = self.wp_parser.get_wps() # array of yaml

        # Define the acceptable legs
        self.gps_legs = ["gps1", "gps2"]
        self.aruco_legs = ["aruco1", "aruco2", "aruco3"]
        self.obj_legs = ["mallet", "bottle"]

        # Initialize variables
        self.leg = "start"
        self.filtered_gps = None

        # Hex pattern for searching
        self.hex_coord = [
            (2.0, 0.0),
            (1.0, 1.73),
            (-1.0, 1.73),
            (-2.0, 0.0),
            (-1.0, -1.73),
            (1.0, -1.73),
        ]
        self.hex_scalar = 0.00005

        ################################
        ### ROS 2 OBJECT DEFINITIONS ###
        ################################

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
        self.teleop_client = self.create_client(Trigger, "trigger_teleop", callback_group=bg_callback_group)
        while not self.teleop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Teleop trigger service not available, waiting again..."
            )
        self.teleop_request = Trigger.Request()

        # Client to trigger autonomy state
        self.nav_client = self.create_client(Trigger, "trigger_auto", callback_group=bg_callback_group)
        while not self.nav_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Autonomy trigger service not available, waiting again..."
            )
        self.nav_request = Trigger.Request()

        # Client to trigger arrival state
        self.arrival_client = self.create_client(Trigger, "trigger_arrival", callback_group=bg_callback_group)
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
            callback_group=fg_callback_group
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
            self, FollowGPSWaypoints, 'follow_gps_waypoints', callback_group=self.cmd_callback_group
        )
        self.spin_client = ActionClient(self, Spin, 'spin', callback_group=self.cmd_callback_group)
        
        ###########################################
        ### END NAV2 BASIC NAVIGATOR BASED CODE ###
        ###########################################

        self.get_logger().info("State machine node initialized")

    #######################################
    ### NAV2 BASIC NAVIGATOR BASED CODE ###
    #######################################

    def followGpsWaypoints(self, gps_poses):
        """
        Function to follow a set of GPS waypoints, based on the nav2_simple_commander code
        """

        self.debug("Waiting for 'FollowWaypoints' action server")
        while not self.follow_gps_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowWaypoints' action server not available, waiting...")

        goal_msg = FollowGPSWaypoints.Goal()
        goal_msg.gps_poses = gps_poses

        self.info(f'Following {len(goal_msg.gps_poses)} gps goals....')
        send_goal_future = self.follow_gps_waypoints_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def spin(self, spin_dist=1.57, time_allowance=10, disable_collision_checks=False):
        """
        Function to spin in place, based on the nav2_simple_commander code
        """

        self.debug("Waiting for 'Spin' action server")
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist
        goal_msg.time_allowance = Duration(sec=time_allowance)
        # goal_msg.disable_collision_checks = disable_collision_checks

        self.info(f'Spinning to angle {goal_msg.target_yaw}....')
        send_goal_future = self.spin_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Spin request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def cancelTask(self):
        """
        Cancel pending task request of any type, based on the nav2_simple_commander code
        """

        self.info('Canceling current task.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return
    
    def isTaskComplete(self):
        """
        Check if the task request of any type is complete yet, based on the nav2_simple_commander code
        """

        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Task succeeded!')
        return True
    
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

    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl'):
        """
        Block until the full navigation system is up and running, based on the nav2_simple_commander code
        """

        if localizer != 'robot_localization':  # non-lifecycle node
            self._waitForNodeToActivate(localizer)
        # if localizer == 'amcl':
        #     self._waitForInitialPose()
        self._waitForNodeToActivate(navigator)
        self.info('Nav2 is ready for use!')
        return
    
    def _waitForNodeToActivate(self, node_name):
        """
        Waits for the node within the tester namespace to become active, based on the nav2_simple_commander code
        """
        
        self.debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service, callback_group=self.cmd_callback_group)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.debug(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f'Result of get_state: {state}')
            time.sleep(2)
        return
    
    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
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

    def action_server_callback(self, goal_handle):
        """
        Callback function for the action server
        """

        self.sm_goal_handle = goal_handle

        # Trigger the autonomy state
        future = self.nav_client.call_async(self.nav_request)
        rclpy.spin_until_future_complete(self, future)

        self.run_behavior_tree()

        # Trigger the teleop state
        future = self.teleop_client.call_async(self.teleop_request)
        rclpy.spin_until_future_complete(self, future)

        self.sm_goal_handle.succeed()
        result = RunBT.Result()
        return result
    
    def gps_callback(self, msg):
        """
        Callback function for the GPS subscriber
        """
        self.filtered_gps = latLonYaw2Geopose(msg.latitude, msg.longitude)

    def aruco_callback(self, msg):
        """
        Callback function for the aruco pose subscriber
        """

        # ArucoDetections:
        #   std_msgs/Header header
        #   aruco_opencv_msgs/MarkerPose[] markers
        #   aruco_opencv_msgs/BoardPose[] boards
        # MarkerPose:
        #   uint16 marker_id
        #   geometry_msgs/Pose pose

        # TODO: Implement this function

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
        sm_feedback.feedback = "[INFO] [" + self.leg + "] " + string
        self.sm_goal_handle.publish_feedback(sm_feedback)

    def bt_warn(self, string):
        """
        Function to write warnings back to the RunBT action client
        """

        self.get_logger().warn("[" + self.leg + "] " + string)
        sm_feedback = RunBT.Feedback()
        sm_feedback.feedback = "[WARN] [" + self.leg + "] " + string
        self.sm_goal_handle.publish_feedback(sm_feedback)

    def bt_error(self, string):
        """
        Function to write errors back to the RunBT action client
        """

        self.get_logger().error("[" + self.leg + "] " + string)
        sm_feedback = RunBT.Feedback()
        sm_feedback.feedback = "[ERROR] [" + self.leg + "] " + string
        self.sm_goal_handle.publish_feedback(sm_feedback)

    def bt_fatal(self, string):
        """
        Function to write fatal errors back to the RunBT action client
        """

        self.get_logger().fatal("[" + self.leg + "] " + string)
        sm_feedback = RunBT.Feedback()
        sm_feedback.feedback = "[FATAL] [" + self.leg + "] " + string
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
        self.bt_info("Executing leg")

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

            # Trigger the arrival state
            future = self.arrival_client.call_async(self.arrival_request)
            rclpy.spin_until_future_complete(self, future)

            time.sleep(5)

            # Trigger the autonomy state
            future = self.nav_client.call_async(self.nav_request)
            rclpy.spin_until_future_complete(self, future)

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
                self.bt_info("Found the aruco tag at: " + aruco_loc)
                self.gps_nav(aruco_loc)

                # Trigger the arrival state
                future = self.arrival_client.call_async(self.arrival_request)
                rclpy.spin_until_future_complete(self, future)

                time.sleep(5)

                # Trigger the autonomy state
                future = self.nav_client.call_async(self.nav_request)
                rclpy.spin_until_future_complete(self, future)

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
            obj_loc = self.spin_search(leg)  # Do a spin search
            if not obj_loc:
                obj_loc = self.hex_search(leg)  # Do a hex search
            if not obj_loc:
                self.bt_error("Could not find the object")
            else:
                self.bt_info("Found the object at: " + obj_loc)
                self.gps_nav(obj_loc)

                # Trigger the arrival state
                future = self.arrival_client.call_async(self.arrival_request)
                rclpy.spin_until_future_complete(self, future)

                time.sleep(5)

                # Trigger the autonomy state
                future = self.nav_client.call_async(self.nav_request)
                rclpy.spin_until_future_complete(self, future)

    def gps_nav(self, dest_wp):
        """
        Function to navigate through GPS waypoints
        """

        self.bt_info("Starting GPS navigation")
        
        # Generate a path from the current GPS location to the end GPS location
        path = basicPathPlanner(self.filtered_gps, dest_wp)

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

        self.followGpsWaypoints(path)
        while not self.isTaskComplete():
            time.sleep(0.1)

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.bt_info("GPS navigation completed")
        elif result == TaskResult.CANCELED:
            self.bt_warn("GPS navigation canceled")
        elif result == TaskResult.FAILED:
            self.bt_error("GPS navigation failed")

    def spin_search(self):
        """
        Function to perform a spin search
        """

        self.bt_info("Starting spin search")

        self.spin(spin_dist=3.14)
        while not self.isTaskComplete():

            if self.leg in self.aruco_legs:
                # Check for the aruco tag
                pose = self.aruco_check()
                if pose:
                    self.cancelTask()
                    return pose

            elif self.leg in self.obj_legs:
                # Check for the object
                pose = self.obj_check()
                if pose:
                    self.cancelTask()
                    return pose

            time.sleep(0.1)

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.bt_info("Spin search completed")
        elif result == TaskResult.CANCELED:
            self.bt_warn("Spin search canceled")
        elif result == TaskResult.FAILED:
            self.bt_error("Spin search failed")

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
        for coord in self.hex_coord:
            hex_lat = base_wp.position.latitude + coord[0] * self.hex_scalar
            hex_lon = base_wp.position.longitude + coord[1] * self.hex_scalar
            hex_wp = latLonYaw2Geopose(hex_lat, hex_lon)

            self.gps_nav(hex_wp)
            pose = self.spin_search() # Do a spin search
            # Did the last spin search find it?
            if pose:
                return pose

        return False

    def aruco_check(self):
        """
        Function to check for the aruco tag
        """

        # TODO: Implement this function

        return False

    def obj_check(self):
        """
        Function to check for the object
        """

        # TODO: Implement this function

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
