#!/bin/bash
# Created by Nelson Durrant, Mar 2025
# 
# Drive the rover in sim using the keyboard

source ~/rover_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
