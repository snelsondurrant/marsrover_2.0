#!/bin/bash
# Created by Nelson Durrant, May 2025
#
# Common functions and variables for base station scripts

ROVER_IP_ADDRESS=192.168.1.120
ROVER_USERNAME=marsrover
ROVER_PASSWORD=thekillpack

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

function checkConnection {
  # Check for a connection to the rover
  ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "echo" &> /dev/null
  if [ $? -ne 0 ]; then
      printError "No available SSH connection to the rover"
      echo "Here's some debugging suggestions:"
      echo "  - Ensure the rover is powered on"
      echo "  - Ensure the rover is connected with a static IP address"
      echo "  - Run 'bash setup_ssh.sh' to set up SSH keys"

      exit 1
  fi
}
