#!/bin/bash
# Created by Nelson Durrant, Feb 2025
# 
# Runs the autonomy task in the real world environment

LOOPBACK_IP_ADDRESS=127.0.0.1
ROVER_IP_ADDRESS=192.168.1.120
FAST_DDS_PORT=11811

# Are we running on Jetson Orin architecture (the rover)?
if [ "$(uname -m)" == "aarch64" ]; then
    export ROS_DISCOVERY_SERVER=$LOOPBACK_IP_ADDRESS:$FAST_DDS_PORT
else
    export ROS_DISCOVERY_SERVER=$ROVER_IP_ADDRESS:$FAST_DDS_PORT
fi

source ~/rover_ws/install/setup.bash
ros2 action send_goal --feedback exec_autonomy_task rover_interfaces/action/RunTask \
    "{legs: ['gps1', 'gps2', 'aruco1', 'aruco2', aruco3', 'mallet', 'bottle']}"
