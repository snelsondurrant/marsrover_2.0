from rover_navigation.utils.gps_utils import latLonYaw2Geopose, quaternion_from_euler


def terrainPathPlanner(geopose1, geopose2, wp_dist):
    """
    Generate intermediary waypoints between two GPS coordinates with terrain consideration

    :author:
    :date:
    """

    # TODO: Implement terrainPathPlanner

    return


def terrainOrderPlanner(legs, fix):
    """
    Select the order in which to visit waypoints based on terrain considerations

    :author:
    :date:
    """

    # TODO: Implement terrainOrderPlanner

    return

# import numpy as np
# import rasterio
# from rasterio.transform import rowcol
# import utm
# from astar import AStar
# from rover_navigation.utils.gps_utils import latLonYaw2Geopose, geopose2LatLonYaw
# from rover_navigation.utils.plan_utils import basicPathPlanner

# # Global raster cache to avoid repeated reads
# _raster_cache = {}

# # Known terrain maps
# _maps = [
#     "/home/marsrover-docker/rover_ws/src/rover_navigation/rover_navigation/maps/byu_campus.tif",
# ]

# def terrainPathPlanner(geopose1, geopose2, wp_dist):
#     """
#     Generate intermediary waypoints between two GPS coordinates considering terrain (elevation data)

#     :author: Nelson Durrant
#     :date: Apr 2025
#     """

#     # Convert geopose to lat/lon and then to UTM coordinates
#     start_lat, start_lon = geopose2LatLonYaw(geopose1)[:2]
#     end_lat, end_lon = geopose2LatLonYaw(geopose2)[:2]
#     start_e, start_n, zone, letter = utm.from_latlon(start_lat, start_lon)
#     end_e, end_n = utm.from_latlon(end_lat, end_lon)[:2]

#     # Loop over all known terrain maps to find one with valid data
#     for map_path in _maps:
#         try:
#             # Open or reuse raster from cache
#             if map_path in _raster_cache:
#                 src = _raster_cache[map_path]
#             else:
#                 src = rasterio.open(map_path)
#                 _raster_cache[map_path] = src

#             # Convert UTM coordinates to raster row/column indices
#             start_rc = rowcol(src.transform, start_e, start_n)
#             end_rc = rowcol(src.transform, end_e, end_n)

#             # Ensure both points are within the bounds of the raster data
#             if all(0 <= r < src.height for r in [start_rc[0], end_rc[0]]) and all(
#                 0 <= c < src.width for c in [start_rc[1], end_rc[1]]
#             ):

#                 # Read elevation data (terrain map)
#                 terrain = src.read(1)

#                 # Use the A* algorithm to find the optimal path considering terrain
#                 path = list(TerrainAStar(terrain).astar(start_rc, end_rc))

#                 # Convert raster path (row/col) back to GPS waypoints
#                 utm_path = [src.transform * (col, row) for row, col in path]

#                 # Interpolate points every wp_dist meters along the path
#                 def interpolate_path(points, spacing):
#                     distances = [0.0]
#                     for i in range(1, len(points)):
#                         dx = points[i][0] - points[i - 1][0]
#                         dy = points[i][1] - points[i - 1][1]
#                         distances.append(distances[-1] + np.hypot(dx, dy))

#                     total_dist = distances[-1]
#                     if total_dist < spacing:
#                         return points

#                     sampled_points = []
#                     current_dist = 0.0
#                     i = 0
#                     while current_dist <= total_dist and i < len(points) - 1:
#                         while distances[i + 1] < current_dist:
#                             i += 1
#                         # Linear interpolation between points
#                         ratio = (current_dist - distances[i]) / (distances[i + 1] - distances[i])
#                         x = points[i][0] + ratio * (points[i + 1][0] - points[i][0])
#                         y = points[i][1] + ratio * (points[i + 1][1] - points[i][1])
#                         sampled_points.append((x, y))
#                         current_dist += spacing

#                     sampled_points.append(points[-1])
#                     return sampled_points

#                 # Sample the path at regular intervals based on wp_dist
#                 sampled_utm_path = interpolate_path(utm_path, wp_dist)

#                 # Convert the sampled UTM points to GPS waypoints (geoposes)
#                 waypoints = [
#                     latLonYaw2Geopose(*utm.to_latlon(e, n, zone, letter)) 
#                     for e, n in sampled_utm_path
#                 ]

#                 return waypoints

#         except Exception:
#             continue

#     # If no valid terrain data is found, fall back to a basic straight-line path planner
#     return basicPathPlanner(geopose1, geopose2, wp_dist)

# class TerrainAStar(AStar):
#     """
#     Terrain-based A* algorithm (adapted to account for elevation differences)
#     https://github.com/jrialland/python-astar

#     :author: Nelson Durrant 
#     :date: Apr 2025
#     """

#     def __init__(self, terrain_map, elev_penalty=0.1):
#         self.terrain_map = terrain_map
#         self.rows, self.cols = terrain_map.shape
#         self.elev_penalty = elev_penalty

#     def neighbors(self, node):
#         r, c = node
#         directions = [
#             (-1, 0),  # North
#             (1, 0),   # South
#             (0, -1),  # West
#             (0, 1),   # East
#             (-1, -1), # Northwest
#             (-1, 1),  # Northeast
#             (1, -1),  # Southwest
#             (1, 1),   # Southeast
#         ]
#         return [
#             (r + dr, c + dc)
#             for dr, dc in directions
#             if 0 <= r + dr < self.rows and 0 <= c + dc < self.cols
#         ]

#     def distance_between(self, n1, n2):
#         r1, c1 = n1
#         r2, c2 = n2
#         dz = abs(self.terrain_map[r2, c2] - self.terrain_map[r1, c1])  # Elevation difference
#         flat = np.hypot(r2 - r1, c2 - c1)  # Euclidean distance on the flat plane
#         return flat + dz * self.elev_penalty  # Total cost considering terrain

#     def heuristic_cost_estimate(self, current, goal):
#         return np.hypot(goal[0] - current[0], goal[1] - current[1])

#     def is_goal_reached(self, current, goal):
#         return current == goal
