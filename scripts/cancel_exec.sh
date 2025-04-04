#!/bin/bash
# Created by Nelson Durrant, Feb 2025
# 
# Cancels the autonomy task in the real world environment

source ~/rover_ws/install/setup.bash
ros2 service call /exec_autonomy_task/_action/cancel_goal action_msgs/srv/CancelGoal "{}"