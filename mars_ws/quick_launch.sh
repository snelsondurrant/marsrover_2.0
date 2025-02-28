#!/bin/bash
# Created by Nelson Durrant, Feb 2025

source ~/mars_ws/install/setup.bash
ros2 run nav2_autonomy state_machine --ros-args -p use_sim_time:=true