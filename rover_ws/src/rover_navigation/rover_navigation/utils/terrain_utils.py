from rover_navigation.utils.gps_utils import latLonYaw2Geopose, quaternion_from_euler


def terrainPathPlanner(geopose1, geopose2, wp_dist):
    """
    Generate intermediary waypoints between two GPS coordinates with terrain consideration

    :author:
    :date:
    """

    # TODO: Implement terrainPathPlanner

    # NOTE: I'd recommend using a GeoTIFF file and the rasterio library to read the terrain data, 
    # converting the lat/lon coordinates to pixel coordinates using the rasterio.transform.rowcol function
    # and the utm library. The astar python library should then be able to use the rasterio maps created to
    # effectively plan a terrain-informed path between the two waypoints. (You should be able to test it 
    # and get it running in simulation first -- Sonoma Raceway is a real place with real terrain data)
    # 
    # Here's some of the relevant documentation:
    # https://rasterio.readthedocs.io/en/stable/quickstart.html#reading-a-geotiff
    # https://rasterio.readthedocs.io/en/stable/api/rasterio.transform.html#rasterio.transform.rowcol
    # https://github.com/Turbo87/utm
    # https://github.com/jrialland/python-astar
    # 
    # Here's a link to where you can download GeoTIFF data (with a student account):
    # https://portal.opentopography.org/raster?opentopoID=OTNED.012021.4269.3
    #
    # - Nelson Durrant, Apr 2025

    return


def terrainOrderPlanner(legs, fix):
    """
    Select the order in which to visit waypoints based on terrain considerations

    :author:
    :date:
    """

    # TODO: Implement terrainOrderPlanner

    return
