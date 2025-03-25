#!/bin/bash
# Created by Braden Meyers, Mar 2025
# 
# Sets up the environment with the ROS_DISCOVERY_SERVER environment variable

function printError {
  	# print red
  	echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

# Check if this script was run using 'source'
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    printError "Please source this script ('source enable_cli.sh')"
    exit 1
fi

ROVER_IP_ADDRESS=192.168.1.120

# Are we running on Jetson Orin architecture (the rover)?
if [ "$(uname -m)" == "aarch64" ]; then
    export ROS_DISCOVERY_SERVER="localhost:11811"
else
    export ROS_DISCOVERY_SERVER="$ROVER_IP_ADDRESS:11811"
fi
