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
    echo "  - Run 'bash setup_ssh.sh' to set up SSH access"

    exit
fi

# Get the current git branch name
export current_branch=$(git branch --show-current)

# Send tmux commands to the rover over SSH
printInfo "Setting up the 'git_sync' tmux session..."
envsubst < tmuxp/git_sync.yaml > tmuxp/tmp/git_sync.yaml
scp tmuxp/tmp/git_sync.yaml marsrover@$ROVER_IP_ADDRESS:~/marsrover/base_scripts/tmuxp/tmp/
ssh marsrover@$ROVER_IP_ADDRESS \
    "export PATH='$PATH:/home/marsrover/.local/bin'; \
    tmuxp load -d /home/marsrover/marsrover/base_scripts/tmuxp/tmp/git_sync.yaml"

# Attach to the 'git_sync' tmux session to view the output (using mosh)
mosh marsrover@$ROVER_IP_ADDRESS -- tmux attach -t git_sync

# Kill the tmux session on exit
ssh marsrover@$ROVER_IP_ADDRESS "tmux kill-session -t git_sync"
