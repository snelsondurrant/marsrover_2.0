#!/bin/bash
# Created by Nelson Durrant, Feb 2025
# 
# Runs the autonomy task in the real world environment

source ~/rover_ws/install/setup.bash
ros2 action send_goal --feedback exec_autonomy_task rover_interfaces/action/RunTask \
    "{legs: ['gps1', 'gps2', 'aruco1', 'aruco2', aruco3', 'mallet', 'bottle']}"
