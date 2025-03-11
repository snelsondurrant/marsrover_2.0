#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Launches tasks on the rover over SSH using the 'rover_launch' tmux session

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
DOCKER_SSH_PORT=2233

# Check for an SSH connection to the rover's Docker container
if ! ssh marsrover-docker@$ROVER_IP_ADDRESS -p $DOCKER_SSH_PORT "echo" &> /dev/null
then
    printError "No available SSH connection to the rover's Docker container"
    echo "Here's some debugging suggestions:"
    echo "  - Ensure the rover is powered on"
    echo "  - Ensure the rover is connected with a static IP address"
    echo "  - Ensure the rover's Docker container is running"

    exit
fi

# Launch the specified task configuration over SSH
case "$1" in
    "autonomy")
        printInfo "Setting up the autonomy task..."
        ssh marsrover-docker@$ROVER_IP_ADDRESS -p $DOCKER_SSH_PORT "tmuxp load -d workspaces/autonomy/rover_autonomy.yaml"
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

# Attach to the 'rover_launch' tmux session
ssh -t -X marsrover-docker@$ROVER_IP_ADDRESS -p $DOCKER_SSH_PORT \
  "tmux send-keys -t rover_launch 'export DISPLAY=$DISPLAY' Enter; \
  tmux attach -t rover_launch"

# Kill the tmux session on exit
ssh marsrover-docker@$ROVER_IP_ADDRESS -p $DOCKER_SSH_PORT 'tmux kill-session -t rover_launch'
