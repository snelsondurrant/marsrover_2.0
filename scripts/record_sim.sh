#!/bin/bash
# Created by Nelson Durrant, Mar 2025
# 
# Records an autonomy task simulation run to a bag file

tag=sim_run_$(date +%Y%m%d_%H%M%S)

source ~/mars_ws/install/setup.bash
ros2 bag record -a -o ~/mars_ws/src/rover_gazebo/bags/sim_run_$tag

ros2 run plotjuggler plotjuggler ~/mars_ws/src/rover_gazebo/bags/sim_run_$tag/sim_run_$tag.db3
