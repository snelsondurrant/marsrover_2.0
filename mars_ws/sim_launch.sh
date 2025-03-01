#!/bin/bash
# Created by Nelson Durrant, Feb 2025

source ~/mars_ws/install/setup.bash
ros2 launch nav2_autonomy full_stack_sim.launch.py use_rviz:=True use_mapviz:=True
