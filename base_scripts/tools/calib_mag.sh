#!/bin/bash
# Created by Nelson Durrant, Mar 2025
#
# Calibrate the ZED camera magnetometer on the rover
# TODO: Test this

function printInfo {
 	# print blue
 	echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  	# print yellow
  	echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  	# print red
  	echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

ROVER_IP_ADDRESS=192.168.1.120
ROVER_USERNAME=marsrover

# Check for a "-u <username>" argument
while getopts ":u:" opt; do
  case $opt in
    u)
      ROVER_USERNAME=$OPTARG
      ;;
  esac
done

ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "echo" &> /dev/null
if [ $? -ne 0 ]; then
    printError "No available SSH connection to the rover"
    echo "Here's some debugging suggestions:"
    echo "  - Ensure the rover is powered on"
    echo "  - Ensure the rover is connected with a static IP address"
    echo "  - Run 'bash setup_ssh.sh' to set up SSH keys"

    exit 1
fi

# Launch the ZED Sensor Viewer over SSH -X
ssh -t -X $ROVER_USERNAME@$ROVER_IP_ADDRESS "cd /usr/local/zed/tools && ./ZED_Sensor_Viewer"
