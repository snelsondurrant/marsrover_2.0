from astar import AStar
from geographic_msgs.msg import GeoPose, GeoPoint
from itertools import permutations
import math
import os
import rasterio
from rasterio.transform import rowcol
import utm


class TerrainGraph(AStar):
    """
    A* graph for terrain navigation using GeoTIFF elevation data.
    https://github.com/jrialland/python-astar

    :author: Nelson Durrant
    :date: Apr 2025
    """

    def __init__(self, elevation_data, transform, elev_cost):
        self.elevation_data = elevation_data
        self.transform = transform
        self.rows, self.cols = elevation_data.shape
        self.elev_cost = elev_cost

    def heuristic_cost_estimate(self, n1, n2):
        # Simple Euclidean distance in pixel space
        return ((n1[0] - n2[0]) ** 2 + (n1[1] - n2[1]) ** 2) ** 0.5

    def neighbors(self, node):
        row, col = node
        neighbors_list = []
        # Get all viable neighbors (8 directions)
        for dr, dc in [
            (0, 1),
            (0, -1),
            (1, 0),
            (-1, 0),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ]:
            new_row, new_col = row + dr, col + dc
            if 0 <= new_row < self.rows and 0 <= new_col < self.cols:
                neighbors_list.append((new_row, new_col))
        return neighbors_list

    def distance_between(self, n1, n2):
        # Simple Euclidean distance in pixel space
        d = math.sqrt((n1[0] - n2[0]) ** 2 + (n1[1] - n2[1]) ** 2)

        # Add cost based on elevation change
        elevation1 = self.elevation_data[n1]
        elevation2 = self.elevation_data[n2]
        elevation_diff = abs(elevation1 - elevation2)

        cost = d + elevation_diff * self.elev_cost
        return cost


def geopose_to_pixel(geopose, transform):
    """
    Converts a geographic_msgs/GeoPose to pixel coordinates.
    """

    lat = geopose.position.latitude
    lon = geopose.position.longitude
    east, north, zone_number, zone_letter = utm.from_latlon(lat, lon)
    row, col = rowcol(transform, east, north)
    row, col = int(round(row)), int(round(col))
    return (row, col), (zone_number, zone_letter)


def pixel_to_geopose(pixel_coords, transform, utm_zone):
    """
    Converts pixel coordinates back to a geographic_msgs/GeoPose.
    """

    row, col = pixel_coords
    east, north = transform * (col + 0.5, row + 0.5)  # Use pixel center
    lat, lon = utm.to_latlon(east, north, utm_zone[0], utm_zone[1])
    new_geopose = GeoPose()
    new_geopose.position = GeoPoint()
    new_geopose.position.latitude = lat
    new_geopose.position.longitude = lon
    return new_geopose


def terrainPathPlanner(start_geopose, end_geopose, elev_cost=1.0):
    """
    Generate intermediary waypoints between two GPS coordinates with terrain consideration

    :author: Nelson Durrant
    :date: Apr 2025
    """

    geotiff_file = None
    geotiff_path = (
        "/home/marsrover-docker/rover_ws/src/rover_navigation/rover_navigation/maps"
    )
    
    # Check all of our maps to see if we have a valid one
    geotiff_files = [f for f in os.listdir(geotiff_path) if f.endswith(".tif")]
    for file in geotiff_files:
        with rasterio.open(os.path.join(geotiff_path, file)) as src:
            transform = src.transform

            # Is our current location included in the map?
            start_pixel, _ = geopose_to_pixel(start_geopose, transform)
            end_pixel, _ = geopose_to_pixel(end_geopose, transform)
            if 0 <= start_pixel[0] < src.height and 0 <= start_pixel[1] < src.width:
                if 0 <= end_pixel[0] < src.height and 0 <= end_pixel[1] < src.width:
                    # We found a valid map!
                    geotiff_file = os.path.join(geotiff_path, file)
                    break

    if not geotiff_file:
        print("No valid GeoTIFF file found in the specified directory.")
        # TODO: Handle this
        return []

    with rasterio.open(geotiff_file) as src:
        elevation_data = src.read(1)
        transform = src.transform

        # Convert to the pixel space
        start_pixel, start_utm_zone = geopose_to_pixel(start_geopose, transform)
        end_pixel, _ = geopose_to_pixel(end_geopose, transform)
        utm_zone = start_utm_zone

        # Initialize and run A*
        terrain_graph = TerrainGraph(elevation_data, transform, elev_cost=1.0)
        path_pixels = terrain_graph.astar(start_pixel, end_pixel)
        if not path_pixels:
            raise Exception("No viable terrain-based path found using AStar.")

        # Convert pixel path to GeoPose path
        path_geoposes = []
        for pixel in path_pixels:
            gp = pixel_to_geopose(pixel, transform, utm_zone)
            path_geoposes.append(gp)

        return path_geoposes


def terrainOrderPlanner(legs, fix):
    """
    Brute force the optimal order to complete the task legs (based on terrain)

    This is an NP-hard problem, but we deal with such small numbers of legs that we can brute force a
    basic optimal solution in a reasonable time.

    :author: Nelson Durrant
    :date: Apr 2025
    """

    lowest_cost = float("inf")
    best_order = []

    # Generate all possible permutations of the task legs
    for order in permutations(legs):

        cost = 0.0  # TODO: Calculate the cost from fix to the first leg

        for i in range(len(order) - 1):

            cost = 0.0  # TODO: Calculate the cost from order[i] to order[i+1]

        # Update the best order
        if cost < lowest_cost:
            lowest_cost = cost
            best_order = order

    return best_order
