"""
Created by Nelson Durrant, Feb 2025

State machine for the BYU Mars Rover using Nav2
Pretty cool, right?
"""

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time

from nav2_gps_waypoint_follower_demo.utils.gps_utils import latLonYaw2Geopose


class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self, leg_id):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            if wp["leg_id"] == leg_id:    
                latitude, longitude = wp["latitude"], wp["longitude"], wp["yaw"] # Need to generate intermittent values somehow
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps


class StateMachine():
    """
    Class to use nav2 gps waypoint follower to follow a set of waypoints logged in a yaml file
    """

    def __init__(self, wps_file_path):
        self.navigator = BasicNavigator("basic_navigator")
        self.wp_parser = YamlWaypointParser(wps_file_path)

        self.gps_legs = ["gps1", "gps2"]
        self.aruco_legs = ["aruco1", "aruco2", "aruco3"]
        self.object_legs = ["mallet", "bottle"]

        self.hex_coord = [(2.0, 0.0), (1.0, 1.73), (-1.0, 1.73), (-2.0, 0.0), (-1.0, -1.73), (1.0, -1.73)]

    def pose_nav(self, pose, leg_id):
        """
        Function to navigate to a pose
        """

        print(leg_id, ': Starting pose navigation')

        self.navigator.goToPose(pose)
        while (not self.navigator.isTaskComplete()):
            time.sleep(0.1)

        result = self.navigator.getTaskResult()
        if result == TaskResult.SUCCEEDED:
            print(leg_id, ': Pose navigation completed')
        elif result == TaskResult.CANCELED:
            print(leg_id, ': Pose navigation canceled')
        elif result == TaskResult.FAILED:
            print(leg_id, ': Pose navigation failed')

    def gps_nav(self, leg_id):
        """
        Function to navigate through GPS waypoints
        """

        print(leg_id, ': Starting GPS navigation')

        wps = self.wp_parser.get_wps(leg_id)
        self.navigator.followGpsWaypoints(wps)
        while (not self.navigator.isTaskComplete()):
            time.sleep(0.1)
        
        result = self.navigator.getTaskResult()
        if result == TaskResult.SUCCEEDED:
            print(leg_id, ': GPS navigation completed')
        elif result == TaskResult.CANCELED:
            print(leg_id, ': GPS navigation canceled')
        elif result == TaskResult.FAILED:
            print(leg_id, ': GPS navigation failed')

    def spin_search(self, leg_id):
        """
        Function to spin in place
        """

        print(leg_id, ': Starting spin search')

        self.navigator.spin(spin_dist=3.14)
        while (not self.navigator.isTaskComplete()):
            # if found the tag or object, cancel, return the loc
            time.sleep(0.1)

        result = self.navigator.getTaskResult()
        if result == TaskResult.SUCCEEDED:
            print(leg_id, ': Spin search completed')
        elif result == TaskResult.CANCELED:
            print(leg_id, ': Spin search canceled')
        elif result == TaskResult.FAILED:
            print(leg_id, ': Spin search failed')
        
        return False

    def hex_search(self, leg_id):
        """
        Function to search in a hex pattern
        """

        print(leg_id, ': Starting hex search')

        # Generate a hex pattern in the base_link frame
        for coord in self.hex_coord:
            pose = PoseStamped()
            pose.header.frame_id = 'base_link'
            pose.header.stamp = navigator.get_clock().now().to_msg()
            pose.pose.position.x = coord[0]
            pose.pose.position.y = coord[1]
            
            pose_nav(pose, leg_id)
            spin_search(leg_id)
            # if found the tag or object, cancel, return the loc
        
        return False

    def aruco_check(self, leg_id):
        """
        Function to check for the aruco tag
        """

        return False

    def object_check(self, leg_id):
        """
        Function to check for the object
        """

        return False

    def run_state_machine(self):
        """
        Function to run the competition state machine
        """
        self.navigator.waitUntilNav2Active(localizer='robot_localization')

        # Iterate through the gps legs
        for gps in gps_legs:

            print(gps, ': Starting GPS leg')
            gps_nav(gps)

        # Iterate through the aruco legs
        for aruco in aruco_legs:

            print(aruco, ': Starting aruco leg')
            gps_nav(aruco)

            # Look for the aruco tag
            aruco_loc = self.spin_search() # Do a spin search
            if not aruco_loc:
                aruco_loc = self.hex_search() # Do a hex search
            if not aruco_loc:
                print(aruco, ': Could not find the aruco tag')
            else:
                print(aruco, ': Found the aruco tag at: ', aruco_loc)
                pose_nav(aruco_loc, aruco)

        # Iterate through the object legs
        for obj in object_legs:

            print(obj, ': Starting object leg')
            gps_nav(obj)

            # Look for the object
            obj_loc = self.spin_search() # Do a spin search
            if not obj_loc:
                obj_loc = self.hex_search() # Do a hex search
            if not obj_loc:
                print(obj, ': Could not find the object')
            else:
                print(obj, ': Found the object at: ', obj_loc)
                pose_nav(obj_loc, obj)


def main():
    rclpy.init()

    # allow to pass the waypoints file as an argument
    default_yaml_file_path = os.path.join(get_package_share_directory(
        "nav2_state_machine"), "waypoints", "comp_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    gps_wpf = GpsWpCommander(yaml_file_path)
    gps_wpf.start_wpf()


if __name__ == "__main__":
    main()
