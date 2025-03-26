#!/bin/bash
# Created by Nelson Durrant, Mar 2025
#
# Run this while the ZED is mounted on the rover to calibrate the ZED's magnetometer
# https://www.stereolabs.com/docs/sensors/magnetometer#magnetometer-calibration

# WARNING! You need to connect using 'ssh -X' for GUI output if running this script remotely

cd /usr/local/zed/tools
./ZED_Sensor_Viewer