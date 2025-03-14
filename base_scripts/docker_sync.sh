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
    printError "No available SSH connection to the rover"
    echo "Here's some debugging suggestions:"
    echo "  - Ensure the rover is powered on"
    echo "  - Ensure the rover is connected with a static IP address"
    echo "  - Run 'bash setup_ssh.sh' to set up SSH access"

    exit
fi

# Pull the arm64 version of the Docker image
# (this will override the existing image, so we'll have to re-pull it later)
printInfo "Pulling the arm64 version of the Docker image..."
docker pull --platform linux/arm64 byuawesomerover/marsrover:latest
docker save byuawesomerover/marsrover:latest | gzip > marsrover.tar.gz
scp marsrover.tar.gz marsrover@$ROVER_IP_ADDRESS:~/marsrover/docker
rm marsrover.tar.gz

# Send tmux commands to the rover over SSH
printInfo "Setting up the 'docker_sync' tmux session..."
envsubst < tmuxp/docker_sync.yaml > tmuxp/tmp/docker_sync.yaml # for $DISPLAY
scp tmuxp/tmp/docker_sync.yaml marsrover@$ROVER_IP_ADDRESS:~/marsrover/base_scripts/tmuxp/tmp/
ssh marsrover@$ROVER_IP_ADDRESS \
    "export PATH='$PATH:/home/marsrover/.local/bin'; \
    tmuxp load -d /home/marsrover/marsrover/base_scripts/tmuxp/tmp/docker_sync.yaml"

# Attach to the 'docker_sync' tmux session to view the output
ssh -t -X marsrover@$ROVER_IP_ADDRESS "tmux attach -t docker_sync"

# Kill the tmux session on exit
ssh marsrover@$ROVER_IP_ADDRESS "tmux kill-session -t docker_sync"

# Pull the amd64 version of the Docker image again
# (this will override the arm64 version we just pulled)
printInfo "Pulling the amd64 version of the Docker image..."
docker pull --platform linux/amd64 byuawesomerover/marsrover:latest
