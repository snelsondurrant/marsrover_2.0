#!/bin/bash
# Created by Nelson Durrant, Mar 2025
# 
# Drive the rover using the keyboard

ROVER_IP_ADDRESS=192.168.1.120
FAST_DDS_PORT=11811

# Are we running on Jetson Orin architecture (the rover)?
if [ "$(uname -m)" == "aarch64" ]; then
    export ROS_DISCOVERY_SERVER=localhost:$FAST_DDS_PORT
else
    export ROS_DISCOVERY_SERVER=$ROVER_IP_ADDRESS:$FAST_DDS_PORT
fi

source ~/rover_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cmd_vel_teleop 
