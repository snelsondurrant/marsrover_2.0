#!/bin/bash
# Created by Nelson Durrant, Feb 2025
# 
# Runs the autonomy task in the simulation environment

source ~/mars_ws/install/setup.bash
ros2 action send_goal --feedback run_autonomy_task rover_interfaces/action/RunTask "{legs: ['gps1', 'aruco1', 'mallet']}"