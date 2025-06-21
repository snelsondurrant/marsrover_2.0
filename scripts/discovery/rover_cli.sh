#!/bin/bash
# Created by Braden Meyers & Nelson Durrant, Mar 2025
# 
# Sets up the rover environment for using the ROS2 CLI tools with Fast DDS
# https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html

function printError {
  	# print red
  	echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

# Check if this script was run using 'source'
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    printError "Please source this script ('source rover_cli.sh')"
    exit 1
fi

# Get the directory of this script
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

export FASTRTPS_DEFAULT_PROFILES_FILE=$script_dir"/config/rover_super_client_config.xml"

ros2 daemon stop
ros2 daemon start
