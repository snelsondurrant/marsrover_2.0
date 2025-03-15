#!/bin/bash
# Created by Nelson Durrant, Mar 2025
# 
# Force the rover to transition to the teleop state so we can control it

source ~/rover_ws/install/setup.bash
ros2 service call /trigger_teleop std_srvs/srv/Trigger "{}"
