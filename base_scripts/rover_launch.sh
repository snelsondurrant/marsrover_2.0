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

# Check for an SSH connection to the rover
if ! ssh marsrover-docker@$ROVER_IP_ADDRESS "echo" &> /dev/null
then
    printError "No available SSH connection to the rover"
    echo "Here's some debugging suggestions:"
    echo "  - Ensure the rover is powered on"
    echo "  - Ensure the rover is connected with a static IP address"

    exit
fi

# Launch the specified task configuration over SSH
case "$1" in
    "autonomy")
        printInfo "Setting up the autonomy task..."
        envsubst < tmuxp/autonomy/rover_autonomy.yaml > tmuxp/tmp/rover_launch.yaml # for $DISPLAY
        ssh marsrover-docker@$ROVER_IP_ADDRESS \
          "docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/tmp/rover_launch.yaml"
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
        printError "No task specified, simply entering the current tmux session..."
        echo "Specify a task using 'bash launch.sh <task>' (ex. 'bash launch.sh autonomy')"
        exit
        ;;
esac

# Attach to the 'rover_launch' tmux session
ssh -t -X marsrover-docker@$ROVER_IP_ADDRESS "docker exec -it marsrover-ct tmux attach -t rover_launch"

# Kill the tmux session on exit
ssh marsrover-docker@$ROVER_IP_ADDRESS "docker exec marsrover-ct tmux kill-session -t rover_launch'
