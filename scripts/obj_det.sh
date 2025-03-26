#!/bin/bash
# Created by Nelson Durrant, Mar 2025
# 
# Enable object detection on the ZED camera

source ~/rover_ws/install/setup.bash
ros2 service call /zed/zed_node/enable_obj_det std_srvs/srv/SetBool "{data: true}"
