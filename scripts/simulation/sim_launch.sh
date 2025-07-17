#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Launches the full autonomy task simulation

source ~/rover_ws/install/setup.bash
ros2 launch rover_bringup rover_autonomy.launch.py sim_mode:=True use_rviz:=True use_mapviz:=False
