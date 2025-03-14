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

# Launch the specified task configuration over SSH
case "$1" in
    "autonomy")
        printInfo "Setting up the autonomy task..."
        envsubst < tmuxp/autonomy/base_autonomy.yaml > tmuxp/tmp/base_launch.yaml # for $DISPLAY
        docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/base_launch.yaml
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

# Attach to the 'base_launch' tmux session
docker exec -it marsrover-ct tmux attach -t base_launch

# Kill the tmux session on exit
docker exec marsrover-ct tmux kill-session -t base_launch
