#!/bin/bash
# Created by Braden Meyers, Mar 2025
# 
# Sets up the environment for using the ROS 2 CLI tools with Fast DDS

ROVER_IP_ADDRESS=192.168.1.120
FAST_DDS_PORT=11811

# Are we running on Jetson Orin architecture (the rover)?
if [ "$(uname -m)" == "aarch64" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="/home/marsrover-docker/rover_ws/rover_super_client_config.xml"
    export ROS_DISCOVERY_SERVER=localhost:$FAST_DDS_PORT
else
    export FASTRTPS_DEFAULT_PROFILES_FILE="/home/marsrover-docker/rover_ws/base_super_client_config.xml"
    export ROS_DISCOVERY_SERVER=$ROVER_IP_ADDRESS:$FAST_DDS_PORT
fi

ros2 daemon stop
ros2 daemon start
