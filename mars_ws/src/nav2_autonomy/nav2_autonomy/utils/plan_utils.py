# Created by Nelson Durrant, Feb 2025
import math
from itertools import permutations
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
    distance = ((end_lat - start_lat) ** 2 + (end_lon - start_lon) ** 2) ** 0.5

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


def bruteOrderPlanner(legs, waypoints, fix):
    """
    Brute force the optimal order to complete the task legs (This is an NP-hard problem)

    :author: Nelson Durrant
    :date: Mar 2025
    """

    lowest_cost = float("inf")
    best_order = []

    # Generate all possible permutations of the task legs
    for order in permutations(legs):

        # Calculate the cost of the current order
        cost = costFunctionStart(fix, order[0], waypoints)
        for i in range(len(order) - 1):
            cost += costFunction(order[i], order[i + 1], waypoints)

        # Update the best order
        if cost < lowest_cost:
            lowest_cost = cost
            best_order = order

    return best_order


def greedyOrderPlanner(legs, waypoints, fix):
    """
    Determine a greedy order to complete the task legs (This is an NP-hard problem)

    :author: Nelson Durrant
    :date: Mar 2025
    """

    order = []
    visited = []

    # Get the leg closest to the current position
    current = None
    min_cost = float("inf")
    for leg in legs:
        cost = costFunctionStart(fix, leg, waypoints)
        if cost < min_cost:
            min_cost = cost
            current = leg

    # Visit the rest of the task legs in order of closest distance
    while len(visited) < len(legs):
        visited.append(current)
        order.append(current)
        min_cost = float("inf")
        for leg in legs:
            if leg not in visited:
                cost = costFunction(current, leg, waypoints)
                if cost < min_cost:
                    min_cost = cost
                    current = leg

    return order


def costFunction(leg1, leg2, waypoints):
    """
    Calculate the cost of moving from one task leg to another
    """

    for wp in waypoints:
        if wp["leg"] == leg1:
            start = wp
        elif wp["leg"] == leg2:
            end = wp

    distance = ((end["latitude"] - start["latitude"]) ** 2 + (end["longitude"] - start["longitude"]) ** 2) ** 0.5

    return distance


def costFunctionStart(fix, leg1, waypoints):
    """
    Calculate the cost of moving from the current position to the first task leg
    """

    for wp in waypoints:
        if wp["leg"] == leg1:
            end = wp

    distance = (
        (end["latitude"] - fix.position.latitude) ** 2 + (end["longitude"] - fix.position.longitude) ** 2
    ) ** 0.5

    return distance
