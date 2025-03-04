#!/bin/bash
# Created by Nelson Durrant, Feb 2025
# 
# Enables the simulation state machine

source ~/mars_ws/install/setup.bash
ros2 action send_goal --feedback run_sm rover_interfaces/action/RunSM "{}"