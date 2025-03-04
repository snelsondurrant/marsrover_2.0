#!/bin/bash
# Created by Nelson Durrant, Feb 2025
# 
# Enables the simulation behavior tree

source ~/mars_ws/install/setup.bash
ros2 action send_goal --feedback run_bt rover_interfaces/action/RunBT "{legs: ['gps1', 'aruco1', 'mallet']}"