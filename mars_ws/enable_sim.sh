#!/bin/bash
# Created by Nelson Durrant, Feb 2025
# 
# Enables the simulation state machine

source ~/mars_ws/install/setup.bash
ros2 service call /nav2_sm/enable std_srvs/srv/SetBool "{data: true}"