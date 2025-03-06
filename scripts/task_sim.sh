#!/bin/bash
# Created by Nelson Durrant, Feb 2025
# 
# Runs the autonomy task in the simulation environment

echo ""
echo -e "\033[0m\033[36mQUICK SIMULATION TIPS:\033[0m"
echo "  - Left click on the turtlebot model ('World' tab, 'Models' dropdown) in Gazebo and click 'Follow' to keep the camera on the robot"
echo "  - Use the toolbar across the top of the Gazebo window to drag and drop obstacles into the robot's path"
echo "  - Click 'Add' on the left toolbar in RViz, navigate to the 'By topic' tab, and select 'arduino_tracker/debug/Image' to see the camera feed with aruco tags labeled"
echo ""

source ~/mars_ws/install/setup.bash
ros2 action send_goal --feedback exec_autonomy_task rover_interfaces/action/RunTask "{legs: ['gps1', 'aruco1', 'mallet']}"
