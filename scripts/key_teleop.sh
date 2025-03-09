#!/bin/bash
# Created by Nelson Durrant, Mar 2025
# 
# Drive the rover using the keyboard

source ~/mars_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cmd_vel_teleop
