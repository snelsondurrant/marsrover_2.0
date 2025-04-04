#!/bin/bash
# Created by Nelson Durrant, Feb 2025
# 
# Runs the autonomy task in the simulation environment

source ~/rover_ws/install/setup.bash
ros2 action send_goal --feedback exec_autonomy_task rover_interfaces/action/AutonomyTask \
    "{legs: [
        {
            name: 'gps1', 
            type: 'gps', 
            latitude: 38.162923, 
            longitude: -122.454987
        },
        {
            name: 'aruco1',
            type: 'aruco', 
            latitude: 38.162958, 
            longitude: -122.455412, 
            tag_id: 1
        },
        {
            name: 'mallet',
            type: 'obj', 
            latitude: 38.162635, 
            longitude: -122.454902, 
            object: 'mallet'
        }
    ]}"
