#!/bin/bash
# Created by Nelson Durrant, Feb 2025
# 
# Runs the autonomy task in the real world environment

source ~/rover_ws/install/setup.bash
ros2 action send_goal --feedback exec_autonomy_task rover_interfaces/action/AutonomyTask \
    "{legs: [
        {
            name: 'gps1',
            type: 'gps',
            latitude: 0.0,
            longitude: 0.0
        },
        {
            name: 'gps2',
            type: 'gps',
            latitude: 0.0,
            longitude: 0.0
        },
        {
            name: 'aruco1',
            type: 'aruco',
            latitude: 0.0,
            longitude: 0.0,
            tag_id: 1
        },
        {
            name: 'aruco2',
            type: 'aruco',
            latitude: 0.0,
            longitude: 0.0,
            tag_id: 2
        },
        {
            name: 'aruco3',
            type: 'aruco',
            latitude: 0.0,
            longitude: 0.0,
            tag_id: 3
        },
        {
            name: 'mallet',
            type: 'obj',
            latitude: 0.0,
            longitude: 0.0,
            object: 'mallet'
        },
        {
            name: 'bottle',
            type: 'obj',
            latitude: 0.0,
            longitude: 0.0,
            object: 'bottle'
        }
    ]}"
