#!/bin/bash
# Created by Braden Meyers, Mar 2025
# 
# Calibrate the position of the ZED and LiDAR in the URDF

source ~/rover_ws/install/setup.bash
ros2 service call /calibrate/zed_lidar std_srvs/srv/Trigger "{}"
