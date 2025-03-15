#!/bin/bash
# Created by Nelson Durrant, Mar 2025
# 
# Simply converts our rover xacro file to a udrf file

source ~/rover_ws/install/setup.bash
ros2 run xacro xacro rover.xacro > rover.urdf
