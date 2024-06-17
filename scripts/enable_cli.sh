#!/bin/bash
# Created by Braden Meyers, Mar 2025
# 
# Sets up the environment for using the ROS 2 CLI tools with Fast DDS

function printError {
  	# print red
  	echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

# Check if this script was run using 'source'
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    printError "Please source this script ('source enable_cli.sh')"
    exit 1
fi

# Are we running on Jetson Orin architecture (the rover)?
if [ "$(uname -m)" == "aarch64" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="~/scripts/config/rover_super_client_config.xml"
else
    export FASTRTPS_DEFAULT_PROFILES_FILE="~/scripts/config/base_super_client_config.xml"
fi

ros2 daemon stop
ros2 daemon start
