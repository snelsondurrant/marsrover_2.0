#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Launches the full simulation stack for the autonomy task

source ~/rover_ws/install/setup.bash
ros2 launch rover_bringup rover_task_autonomy.launch.py sim_mode:=True use_rviz:=True use_mapviz:=True
