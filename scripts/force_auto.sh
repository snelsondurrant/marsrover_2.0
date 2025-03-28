#!/bin/bash
# Created by Nelson Durrant, Mar 2025
# 
# Force the rover to transition to the auto state so Nav2 can control it

source ~/rover_ws/install/setup.bash
ros2 service call /trigger_auto std_srvs/srv/Trigger "{}"
