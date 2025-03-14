#!/bin/bash
# Created by Braden Meyers, Feb 2025
#
# Launches the ZED over SSH using the 'zed_launch' tmux session

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

# Send tmux commands to the rover over SSH
printInfo "Setting up the 'zed_launch' tmux session..."
envsubst < tmuxp/autonomy/zed_launch.yaml > tmuxp/tmp/zed_launch.yaml
scp tmuxp/tmp/zed_launch.yaml marsrover@$ROVER_IP_ADDRESS:~/marsrover/base_scripts/tmuxp/tmp/
ssh marsrover@$ROVER_IP_ADDRESS \
	"export PATH='$PATH:/home/marsrover/.local/bin'; \
	tmuxp load -d /home/marsrover/marsrover/base_scripts/tmuxp/tmp/zed_launch.yaml"

# Attach to the 'zed_launch' tmux session to view the output
ssh -t -X marsrover@$ROVER_IP_ADDRESS "tmux attach -t zed_launch"

# Kill the tmux session on exit
ssh marsrover@$ROVER_IP_ADDRESS "tmux kill-session -t zed_launch"

# TODO: Get the ZED running in a Docker container
