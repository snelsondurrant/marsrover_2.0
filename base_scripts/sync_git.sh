#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Pulls the base station local git branch commit state onto the rover

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

# Check for an SSH connection to the rover
if ! ssh marsrover@$ROVER_IP_ADDRESS "echo" &> /dev/null
then
    printError "No available SSH connection to the rover"
    echo "Here's some debugging suggestions:"
    echo "  - Ensure the rover is powered on"
    echo "  - Ensure the rover is connected with a static IP address"

    exit
fi

# Get the current git branch name
current_branch=$(git branch --show-current)

# Send tmux commands to the rover over SSH
printInfo "Setting up the 'sync_git' tmux session..."
envsubst < tmuxp/sync_git.yaml > tmuxp/tmp/sync_git.yaml # for $DISPLAY and $current_branch
ssh marsrover@$ROVER_IP_ADDRESS \
    "tmuxd load -d /home/marsrover/marsrover/base_scripts/tmuxp/tmp/sync_git.yaml"

# Attach to the 'sync_git' tmux session to view the output
ssh -t -X marsrover@$ROVER_IP_ADDRESS "tmux attach -t sync_git"

# Kill the tmux session on exit
ssh marsrover@$ROVER_IP_ADDRESS "tmux kill-session -t sync_git"
