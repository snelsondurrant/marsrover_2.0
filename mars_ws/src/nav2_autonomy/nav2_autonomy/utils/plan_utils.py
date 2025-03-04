# Created by Nelson Durrant, Feb 2025
import math
from nav2_autonomy.utils.gps_utils import latLonYaw2Geopose, quaternion_from_euler


def basicPathPlanner(geopose1, geopose2):
        """
        Generate intermediary waypoints in a straight line between two GPS coordinates

        :author: Nelson Durrant
        :date: Mar 2025
        """

        # Distance between intermediary waypoints (in lat/lon degrees)
        # If the waypoints are too far apart, they won't be in the global costmap
        # and the navigation2 stack won't be able to plan a path between them
        STEP_SIZE = 0.0001

        new_wps = []

        # Get starting waypoint GPS coordinates
        start_lat = geopose1.position.latitude
        start_lon = geopose1.position.longitude
        end_lat = geopose2.position.latitude
        end_lon = geopose2.position.longitude

        # Calculate the desire yaw angle for movement
        yaw = math.atan((end_lat - start_lat) / (end_lon - start_lon))
        if end_lon < start_lon:
            yaw += math.pi

        # Calculate the distance between the two points
        distance = ((end_lat - start_lat)**2 + (end_lon - start_lon)**2)**0.5

        # Calculate the number of intermediary waypoints
        num_waypoints = int(distance / STEP_SIZE)

        if num_waypoints != 0:
            
            # Calculate the step size for each intermediary waypoint
            step_lat = (end_lat - start_lat) / num_waypoints
            step_lon = (end_lon - start_lon) / num_waypoints

            # Generate intermediary waypoints
            for i in range(1, num_waypoints):
                lat = start_lat + i * step_lat
                lon = start_lon + i * step_lon
                
                geopose = latLonYaw2Geopose(lat, lon, yaw)

                new_wps.append(geopose)

        # Add the original waypoint
        geopose2.orientation = quaternion_from_euler(0.0, 0.0, yaw)
        new_wps.append(geopose2)

        return new_wps