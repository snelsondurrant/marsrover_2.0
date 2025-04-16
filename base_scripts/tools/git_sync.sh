#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Pulls the base station's local git branch onto the rover

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

# Check for an SSH connection to the rover
if ! ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "echo" &> /dev/null
then
    printError "No available SSH connection to the rover"
    echo "Here's some debugging suggestions:"
    echo "  - Ensure the rover is powered on"
    echo "  - Ensure the rover is connected with a static IP address"
    echo "  - Run 'bash setup_ssh.sh' to set up SSH keys"

    exit 1
fi

# Check for the git upstream 'base'
if ! ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "cd ~/marsrover_2.0 && git remote -v" | grep -q "base"
then
    printError "No upstream git remote 'base' found on the rover"
    echo "Please run 'bash setup_git.sh' to set up the upstream git remote"

    exit 1
fi

# Get the current git branch name
export current_branch=$(git branch --show-current)

ssh -t marsrover@$ROVER_IP_ADDRESS "cd ~/marsrover_2.0 && git checkout $current_branch"
ssh -t marsrover@$ROVER_IP_ADDRESS "cd ~/marsrover_2.0 && git pull base ${current_branch}"

