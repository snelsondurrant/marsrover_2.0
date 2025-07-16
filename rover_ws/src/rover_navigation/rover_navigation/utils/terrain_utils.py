from astar import AStar
from geographic_msgs.msg import GeoPose, GeoPoint
from itertools import permutations
import math
import os
import rasterio
from rasterio.transform import rowcol
from rover_navigation.utils.gps_utils import latLonYaw2Geopose, latLon2Meters
import utm


class TerrainGraph(AStar):
    """
    A* graph for terrain navigation using GeoTIFF elevation data.
    https://github.com/jrialland/python-astar

    :author: Nelson Durrant
    :date: Apr 2025
    """

    def __init__(self, elevation_data, transform, elev_cost, elev_limit, roll_cost, roll_limit):
        self.elevation_data = elevation_data
        self.transform = transform
        self.rows, self.cols = elevation_data.shape
        self.elev_cost = elev_cost
        self.elev_limit = elev_limit
        self.roll_cost = roll_cost
        self.roll_limit = roll_limit

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

        # Find the left and right neighbors (based on where we are moving)
        if n1[0] == n2[0]:
            # Moving horizontally
            left_neighbor = (n1[0], n1[1] - 1)
            right_neighbor = (n1[0], n1[1] + 1)
        elif n1[1] == n2[1]:
            # Moving vertically
            left_neighbor = (n1[0] - 1, n1[1])
            right_neighbor = (n1[0] + 1, n1[1])
        else:
            # Moving diagonally
            if n1[0] < n2[0]:
                left_neighbor = (n1[0], n1[1] - 1)
                right_neighbor = (n1[0] + 1, n1[1])
            else:
                left_neighbor = (n1[0] + 1, n1[1])
                right_neighbor = (n1[0], n1[1] - 1)

        roll_diff = 0

        # Check if the left and right neighbors are valid
        if (
            0 <= left_neighbor[0] < self.rows
            and 0 <= left_neighbor[1] < self.cols
            and 0 <= right_neighbor[0] < self.rows
            and 0 <= right_neighbor[1] < self.cols
        ):
            # Calculate the elevation difference between the left and right neighbors
            left_elevation = self.elevation_data[left_neighbor]
            right_elevation = self.elevation_data[right_neighbor]
            roll_diff = abs(left_elevation - right_elevation)
            
        if elevation_diff > self.elev_limit:
            # If it's too steep, report an infinite cost
            return float("inf")
        
        if roll_diff > self.roll_limit:
            # If the roll is too steep, report an infinite cost
            return float("inf")

        cost = d + elevation_diff * self.elev_cost + roll_diff * self.roll_cost
        return cost


def geopose_to_pixel(geopose, transform):
    """
    Converts a geographic_msgs/GeoPose to pixel coordinates.
    """

    lat = geopose.position.latitude
    lon = geopose.position.longitude
    east, north, zone_number, zone_letter = utm.from_latlon(lat, lon)
    # https://rasterio.readthedocs.io/en/stable/api/rasterio.transform.html#rasterio.transform.rowcol
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


def downsample_points(num_points, des_dist):
    """
    Downsamples points such that the points are approximately evenly spaced, with the spacing being
    less than or equal to the desired distance (assumes points are fairly evenly spaced).
    """

    path_length = num_points - 1

    # If the path is short enough, just return the endpoint
    if des_dist >= path_length:
        return [num_points - 1]

    num_intervals = math.ceil(path_length / des_dist)
    actual_spacing = path_length / num_intervals
    selected_indices_set = set()

    for i in range(1, num_intervals + 1):
        # Calculate the ideal position along the path (0 to path_length)
        ideal_position = i * actual_spacing
        # Round to the nearest integer index
        index = round(ideal_position)
        index = max(0, min(num_points - 1, index))
        selected_indices_set.add(index)

    # Convert the set to a sorted list for ordered output
    return sorted(list(selected_indices_set))


def terrainPathPlanner(start_geopose, end_geopose, wp_dist, elev_cost, elev_limit, roll_cost, roll_limit):
    """
    Generate intermediary waypoints between two GPS coordinates with terrain consideration

    :author: Nelson Durrant
    :date: Apr 2025
    """

    # NOTE: You can add a new GeoTIFF map by adding a file from this link to the maps folder:
    # https://portal.opentopography.org/raster?opentopoID=OTNED.012021.4269.3

    geotiff_file = None
    geotiff_path = (
        "/home/marsrover-docker/rover_ws/src/rover_navigation/rover_navigation/maps"
    )

    # Check all of our maps to see if we have a valid one
    geotiff_files = [f for f in os.listdir(geotiff_path) if f.endswith(".tif")]
    for file in geotiff_files:
        # https://rasterio.readthedocs.io/en/stable/quickstart.html#opening-a-dataset-in-reading-mode
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
        raise Exception("No viable map found for terrain-based planning")

    # https://rasterio.readthedocs.io/en/stable/quickstart.html#opening-a-dataset-in-reading-mode
    with rasterio.open(geotiff_file) as src:
        elevation_data = src.read(1)
        transform = src.transform

        # Convert to the pixel space
        start_pixel, start_utm_zone = geopose_to_pixel(start_geopose, transform)
        end_pixel, _ = geopose_to_pixel(end_geopose, transform)
        utm_zone = start_utm_zone

        # Initialize and run A*
        terrain_graph = TerrainGraph(elevation_data, transform, elev_cost, elev_limit, roll_cost, roll_limit)
        path_pixels = terrain_graph.astar(start_pixel, end_pixel)
        if not path_pixels:
            raise Exception("No viable path found by terrain-based AStar planner")

        # Convert pixel path to GeoPose path
        path_geoposes = []
        for pixel in path_pixels:
            gp = pixel_to_geopose(pixel, transform, utm_zone)
            path_geoposes.append(gp)
        path_geoposes[-1] = end_geopose

        # Downsample the path to evenly spaced waypoints
        final_path_result = path_geoposes
        num_points = len(path_geoposes)
        if num_points >= 2:
            selected_indices = downsample_points(num_points, wp_dist)
            downsampled_geoposes = [path_geoposes[i] for i in selected_indices]
            final_path_result = downsampled_geoposes

        return final_path_result
