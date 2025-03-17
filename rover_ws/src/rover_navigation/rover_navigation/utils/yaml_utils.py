import yaml


def waypointParser(yaml_path: str) -> list:
    """
    Parse a set of GPS waypoints from a yaml file

    :author: Nelson Durrant
    :date: Mar 2025
    """

    with open(yaml_path, "r") as wps_file:
        wps_dict = yaml.safe_load(wps_file)

    gps_wps = []
    for wp in wps_dict["waypoints"]:
        gps_wps.append(wp)

    return gps_wps
