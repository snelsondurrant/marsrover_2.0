#!/bin/bash
# Created by Nelson Durrant, Mar 2025
# 
# Drive the rover using the keyboard

source ~/mars_ws/install/setup.bash
ros2 run topic_tools relay /cmd_vel_teleop /cmd_vel & # remap to just /cmd_vel for simulation
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cmd_vel_teleop 

kill %1 # Kill the topic relay process
