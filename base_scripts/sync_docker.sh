#!/bin/bash
# Created by Nelson Durrant, Mar 2025
#
# Pulls the base station docker image onto the rover

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
    printError "No available SSH connection to the rover's computer"
    echo "Here's some debugging suggestions:"
    echo "  - Ensure the rover is powered on"
    echo "  - Ensure the rover is connected with a static IP address"

    exit
fi

# Zip and send the base station version of the Docker image to the rover
docker save byuawesomerover/marsrover:latest | gzip > marsrover.tar.gz
scp marsrover.tar.gz marsrover@$ROVER_IP_ADDRESS:~/marsrover/docker
rm marsrover.tar.gz

# Send tmux commands to the rover over SSH
printInfo "Setting up the sync_docker tmux session..."
ssh marsrover@$ROVER_IP_ADDRESS "tmux new-session -d -s sync_docker; \
    tmux set-option -g default-terminal "screen-256color"; \
    tmux set -g mouse on; \
    tmux send-keys -t sync_docker.0 'clear' Enter; \
    tmux send-keys -t sync_docker.0 'cd ~/marsrover/docker' Enter; \
    tmux send-keys -t sync_docker.0 'gunzip marsrover.tar.gz' Enter; \
    tmux send-keys -t sync_docker.0 'docker load < marsrover.tar' Enter; \
    tmux send-keys -t sync_docker.0 'rm marsrover.tar.gz' Enter; \
    tmux send-keys -t sync_docker.0 'rm marsrover.tar' Enter"

# Attach to the 'sync_docker' tmux session to view the output
ssh -t -X marsrover@$ROVER_IP_ADDRESS \
    "tmux send-keys -t sync_docker 'export DISPLAY=$DISPLAY' Enter; \
    tmux attach -t sync_docker"

# Kill the tmux session on exit
ssh marsrover@$ROVER_IP_ADDRESS "tmux kill-session -t sync_docker"

# TODO: I'm not sure if this'll work with the different processors, but there's got to be a way
