#!/bin/bash
# Created by Nelson Durrant, Mar 2025
# 
# Enable object detection on the ZED camera
# https://www.stereolabs.com/docs/ros2/object-detection#enable-object-detection

source ~/rover_ws/install/setup.bash
ros2 service call /zed/zed_node/enable_obj_det std_srvs/srv/SetBool "{data: true}"
