#!/bin/bash
# Created by Nelson Durrant, Feb 2025
# 
# Records an autonomy task simulation run to a bag file

source ~/mars_ws/install/setup.bash
ros2 bag record -a -o ~/mars_ws/src/nav2_autonomy/bags/sim_run_$(date +%Y%m%d_%H%M%S)

ros2 run plotjuggler plotjuggler ~/mars_ws/src/nav2_autonomy/bags/sim_run_$(date +%Y%m%d_%H%M%S).db3
