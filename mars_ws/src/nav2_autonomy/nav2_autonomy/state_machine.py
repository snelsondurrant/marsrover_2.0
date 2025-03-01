"""
Created by Nelson Durrant, Feb 2025

State machine for the BYU Mars Rover using Nav2
Pretty cool, right?
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import SetBool
from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose
from vision_msgs.msg import Detection3DArray, ObjectHypothesisWithPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import yaml
import time
import threading

from nav2_autonomy.utils.gps_utils import latLonYaw2Geopose

# Globals (interface between ROS node and state machine)
wps_file_path = ""
run_flag = False
found_flag = False
tags = []
wps = []


class YamlArucoWaypointParser:
    """
    Parse a set of legs, aruco tags, and GPS waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, "r") as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_legs(self):
        """
        Get an array of leg names from the yaml file
        """
        return self.wps_dict["legs"]

    def get_wps(self, leg):
        """
        Get an array of ids and geographic_msgs/msg/GeoPose objects from the yaml file
        """
        aruco_tags = []
        for tag in self.wps_dict["aruco_tags"]:
            if tag["leg"] == leg:
                aruco_tags.append(tag)
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            if wp["leg"] == leg:
                latitude, longitude = (
                    wp["latitude"],
                    wp["longitude"],
                )  # Need to generate intermittent values somehow
                gepose_wps.append(latLonYaw2Geopose(latitude, longitude))
        return aruco_tags, gepose_wps


class StateMachine(Node):
    """
    Class for running the autonomy competition task using nav2

    :author: Nelson Durrant
    :date: Feb 2025

    Subscribers:
        - /aruco_detections (aruco_opencv_msgs/ArucoDetection)
        - /zed/detections
    Publishers:
        - /mapviz/goal (sensor_msgs/NavSatFix)
        - /mapviz/inter (sensor_msgs/NavSatFix)
    Services:
        - /nav2_sm/enable (std_srvs/SetBool)
    """

    def __init__(self):

        super().__init__("nav2_state_machine")
        self.navigator = BasicNavigator()

        self.declare_parameter("wps_file_path", "")
        if not self.get_parameter("wps_file_path").value:
            self.get_logger().fatal("No waypoint file path provided")
            rclpy.shutdown()

        wps_file_path = self.get_parameter("wps_file_path").value
        self.wp_parser = YamlArucoWaypointParser(wps_file_path)

        self.legs = self.wp_parser.get_legs()
        self.gps_legs = ["start", "gps1", "gps2"]
        self.aruco_legs = ["aruco1", "aruco2", "aruco3"]
        self.obj_legs = ["mallet", "bottle"]

        self.run_flag = False
        self.found_flag = False
        self.tags = []
        self.wps = []

        self.hex_coord = [
            (2.0, 0.0),
            (1.0, 1.73),
            (-1.0, 1.73),
            (-2.0, 0.0),
            (-1.0, -1.73),
            (1.0, -1.73),
        ]
        self.hex_scalar = 0.00001

        # Aruco pose subscriber
        aruco_callback_group = MutuallyExclusiveCallbackGroup()
        self.aruco_subscriber = self.create_subscription(
            ArucoDetection,
            "/aruco_detections",
            self.aruco_callback,
            10,
            callback_group=aruco_callback_group,
        )
        self.aruco_subscriber  # prevent unused variable warning

        # TODO: Add object detection subscriber

        # Mapviz publishers (to show the goals in mapviz)
        self.mapviz_goal_publisher = self.create_publisher(NavSatFix, "/mapviz/goal", 10)
        self.mapviz_inter_publisher = self.create_publisher(NavSatFix, "/mapviz/inter", 10)

        # Enable service
        enable_callback_group = MutuallyExclusiveCallbackGroup()
        self.enable_service = self.create_service(
            SetBool,
            "/nav2_sm/enable",
            self.enable_callback,
            callback_group=enable_callback_group,
        )

        self.get_logger().info("State machine node initialized")

    def enable_callback(self, request, response):
        """
        Callback function for the enable service
        """

        # Enable or disable the state machine
        if request.data:
            self.run_flag = True
            self.get_logger().info("State machine enabled")
        else:
            self.run_flag = False
            self.get_logger().info("State machine disabled")

        response.success = True
        response.message = (
            "State machine enabled" if request.data else "State machine disabled"
        )

        return response

    def aruco_callback(self, msg):
        """
        Callback function for the aruco pose subscriber
        """

        # Message structure:
        #   ArucoDetections:
        #       std_msgs/Header header
        #       aruco_opencv_msgs/MarkerPose[] markers
        #       aruco_opencv_msgs/BoardPose[] boards
        #   MarkerPose:
        #       uint16 marker_id
        #       geometry_msgs/Pose pose

        for marker in msg.markers:
            if marker.marker_id in self.tags:
                self.get_logger().info(
                    "Found aruco tag: " + marker.marker_id, throttle_duration_sec=1
                )
                self.found_flag = True
                self.found_pose = marker.pose  # TODO: Convert to GeoPose ?

    def gps_nav(self, leg):
        """
        Function to navigate through GPS waypoints
        """

        self.get_logger().info(leg + " Starting GPS navigation")

        # Store relevant tags and waypoints
        self.tags, self.wps = self.wp_parser.get_wps(leg)

        for wp in self.wps:
            # Publish the GPS position in leg to mapviz
            navsat_fix = NavSatFix()
            navsat_fix.header.frame_id = "map"
            navsat_fix.header.stamp = self.navigator.get_clock().now().to_msg()
            navsat_fix.latitude = wp.position.latitude
            navsat_fix.longitude = wp.position.longitude
            self.mapviz_inter_publisher.publish(navsat_fix)

        # Publish the last GPS position in leg (our goal) to mapviz
        navsat_fix = NavSatFix()
        navsat_fix.header.frame_id = "map"
        navsat_fix.header.stamp = self.navigator.get_clock().now().to_msg()
        navsat_fix.latitude = self.wps[-1].position.latitude
        navsat_fix.longitude = self.wps[-1].position.longitude
        self.mapviz_goal_publisher.publish(navsat_fix)

        self.navigator.followGpsWaypoints(self.wps)
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(leg + " GPS navigation completed")
        elif result == TaskResult.CANCELED:
            self.get_logger().warn(leg + " GPS navigation canceled")
        elif result == TaskResult.FAILED:
            self.get_logger().error(leg + " GPS navigation failed")

    def spin_search(self, leg):
        """
        Function to spin in place
        """

        self.get_logger().info(leg + " Starting spin search")

        self.navigator.spin(spin_dist=3.14)
        while not self.navigator.isTaskComplete():

            if leg in self.aruco_legs:
                # Check for the aruco tag
                pose = self.aruco_check(leg)
                if pose:
                    self.navigator.cancelTask()
                    return pose

            elif leg in self.obj_legs:
                # Check for the object
                pose = self.obj_check(leg)
                if pose:
                    self.navigator.cancelTask()
                    return pose

            time.sleep(0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(leg + " Spin search completed")
        elif result == TaskResult.CANCELED:
            self.get_logger().warn(leg + " Spin search canceled")
        elif result == TaskResult.FAILED:
            self.get_logger().error(leg + " Spin search failed")

        return False

    def hex_search(self, leg):
        """
        Function to search in a hex pattern
        """

        self.get_logger().info(leg + " Starting hex search")

        # Generate a hex pattern in the base_link frame
        for coord in self.hex_coord:
            # Convert the hex pattern to GPS points using our last goal
            hex_lat = self.wps[-1].position.latitude + coord[0] * self.hex_scalar
            hex_lon = self.wps[-1].position.longitude + coord[1] * self.hex_scalar
            hex_pose = latLonYaw2Geopose(hex_lat, hex_lon)

            self.gps_nav(hex_pose, leg)
            found_pose = self.spin_search(leg)
            # Did the last spin search find it?
            if found_pose:
                return found_pose

        return False

    def aruco_check(self, leg):
        """
        Function to check for the aruco tag
        """

        if self.found_flag:
            self.found_flag = False
            return self.found_pose
        else:
            return False

    def obj_check(self, leg):
        """
        Function to check for the object
        """

        # Return fake data for the object location
        fake_obj_lat = self.wps[-1].position.latitude + 2 * self.hex_scalar
        fake_obj_lon = self.wps[-1].position.longitude
        pose = latLonYaw2Geopose(fake_obj_lat, fake_obj_lon)

        return pose

    def exec_leg(self, leg):
        """
        Function to execute task legs
        """
        self.navigator.waitUntilNav2Active(localizer="robot_localization")

        # Iterate through the GPS legs
        if leg in self.gps_legs:

            self.get_logger().info(leg + " Starting GPS leg")
            self.gps_nav(leg)

            # Don't wait or flash the LED for the start leg
            if leg == "start":
                return

            # TODO: Wait and flash LED
            time.sleep(5)

        # Iterate through the aruco legs
        elif leg in self.aruco_legs:

            self.get_logger().info(leg + " Starting aruco leg")
            self.gps_nav(leg)

            # Look for the aruco tag
            aruco_loc = self.spin_search(leg)  # Do a spin search
            if not aruco_loc:
                aruco_loc = self.hex_search(leg)  # Do a hex search
            if not aruco_loc:
                self.get_logger().error(leg + " Could not find the aruco tag")
            else:
                self.get_logger().info(leg + " Found the aruco tag at:", aruco_loc)
                self.gps_nav(aruco_loc, leg)

                # TODO: Wait and flash LED
                time.sleep(5)

        # Iterate through the object legs
        elif leg in self.obj_legs:

            self.get_logger().info(leg + " Starting object leg")
            self.gps_nav(leg)

            # Look for the object
            obj_loc = self.spin_search(leg)  # Do a spin search
            if not obj_loc:
                obj_loc = self.hex_search(leg)  # Do a hex search
            if not obj_loc:
                self.get_logger().error(leg + " Could not find the object")
            else:
                self.get_logger().info(leg + " Found the object at:", obj_loc)
                self.gps_nav(obj_loc, leg)

                # TODO: Wait and flash LED
                time.sleep(5)

    def run_state_machine(self):
        """
        Function to run the competition state machine
        """
        while not self.run_flag:
            time.sleep(1)

        for leg in self.legs:
            if not self.run_flag:
                break
            self.exec_leg(leg)

        self.get_logger().info("State machine completed")


def spin_in_thread(exec):
    """
    Function to spin the executor in a separate thread
    """
    exec.spin()


def main(args=None):
    rclpy.init(args=args)

    nav2_sm = StateMachine()
    # Create a multi-threaded executor for Nav2 locking issues
    executor = MultiThreadedExecutor()
    executor.add_node(nav2_sm)

    # Create a separate thread for spinning the executor so we can run the state machine
    spin_thread = threading.Thread(target=spin_in_thread, args=(executor,))
    spin_thread.start()

    nav2_sm.run_state_machine()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    nav2_sm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
