#!/bin/bash
# Created by Nelson Durrant, Mar 2025
# 
# Drive the rover or turtlebot using the keyboard

source ~/rover_ws/install/setup.bash
ros2 run topic_tools relay /cmd_vel_teleop /cmd_vel & # remap to just /cmd_vel for simulation
relay_pid=$! # save the pid of the relay process
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cmd_vel_teleop 

pkill -P $relay_pid # kill the relay process
