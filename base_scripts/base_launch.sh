#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Launches tasks on the base station over SSH using the 'base_launch' tmux session

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

LOOPBACK_IP_ADDRESS=127.0.0.1
ROVER_IP_ADDRESS=192.168.1.120
DOCKER_SSH_PORT=2233

# Check for an SSH connection to the base station's Docker container
if ! ssh marsrover-docker@$LOOPBACK_IP_ADDRESS -p $DOCKER_SSH_PORT "echo" &> /dev/null
then
    printError "No available SSH connection to the base station's Docker container"
    echo "Here's some debugging suggestions:"
    echo "  - Ensure the rover's Docker container is running"

    exit
fi

# Launch the specified task configuration over SSH
case "$1" in
    "autonomy")
        printInfo "Setting up the autonomy task..."
        ssh marsrover-docker@$LOOPBACK_IP_ADDRESS -p $DOCKER_SSH_PORT "tmuxp load -d workspaces/autonomy/base_autonomy.yaml"
        ;;
    "servicing")
        printWarning "Not implemented yet"
        exit
        ;;
    "retrieval")
        printWarning "Not implemented yet"
        exit
        ;;
    "science")
        printWarning "Not implemented yet"
        exit
        ;;
    *)
        printWarning "No task specified, simply entering the current tmux session..."
        echo "Specify a task using 'bash launch.sh <task>' (ex. 'bash launch.sh autonomy')"
        ;;
esac

# Attach to the 'base_launch' tmux session
ssh -t -X marsrover-docker@$LOOPBACK_IP_ADDRESS -p $DOCKER_SSH_PORT 'tmux attach -t base_launch'

# Kill the tmux session on exit
ssh marsrover-docker@$LOOPBACK_IP_ADDRESS -p $DOCKER_SSH_PORT 'tmux kill-session -t base_launch'
