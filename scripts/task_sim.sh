#!/bin/bash
# Created by Nelson Durrant, Feb 2025
# 
# Runs the autonomy task in the simulation environment

echo ""
echo "QUICK SIMULATION TIPS:"
echo "  - Left click on the turtlebot model ('World' tab, 'Models' dropdown) in Gazebo and click 'Follow' to keep the camera fixed on the robot"
echo "  - Click 'Fixed Frame' in the RViz toolbar and select 'base_link' to keep the visualizer centered on the robot frame"
echo "  - Use the toolbar across the top of the Gazebo window to drag and drop obstacles into the robot's path"
echo "  - Click 'Add' on the left toolbar in RViz, navigate to the 'By topic' tab, and select 'arduino_tracker/debug/Image' to see the camera feed with aruco tags labeled"
echo "  - Click 'Add' on the left toolbar in RViz, select 'Detection3DArray' in the 'By display type' tab, and rename the topic in the dropdown to 'zed/detections' to see the object detection bounding boxes"
echo ""
echo "---"
echo ""

source ~/mars_ws/install/setup.bash
ros2 action send_goal --feedback exec_autonomy_task rover_interfaces/action/RunTask "{legs: ['gps1', 'aruco1', 'mallet']}"
