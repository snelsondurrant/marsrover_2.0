#!/bin/bash
# Created by Nelson Durrant, July 2025
#
# Launches the CougUV localization stack for rosbag2 development

source ~/rover_ws/install/setup.bash
ros2 launch coug_bringup coug_dev.launch.py
