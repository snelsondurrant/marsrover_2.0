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

ROVER_USERNAME=marsrover
ROVER_IP_ADDRESS=192.168.1.120

# Check for a "-t <task>" or "-u <username>" argument
while getopts ":t:u:" opt; do
  case $opt in
    u)
      ROVER_USERNAME=$OPTARG
      ;;
    t)
      task=$OPTARG
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

# Start the Docker container if not already running
if [! ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "docker ps" | grep -q "marsrover-ct" ] then
    printWarning "Starting the marsrover-ct container on the rover..."
    ssh -t $ROVER_USERNAME@$ROVER_IP_ADDRESS "cd ~/marsrover_2.0/docker && docker compose up -d"
fi

# Launch the specified task configuration over SSH
case $task in
    "autonomy")
        printInfo "Setting up the autonomy task..."
        envsubst < tmuxp/autonomy/rover_launch.yaml > tmuxp/tmp/rover_launch.yaml
        scp tmuxp/tmp/rover_launch.yaml $ROVER_USERNAME@$ROVER_IP_ADDRESS:~/marsrover_2.0/base_scripts/tmuxp/tmp/
        ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS \
          "docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/rover_launch.yaml"
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
        printError "No task specified"
        echo "Specify a task using 'bash launch.sh -t <task>' (ex. 'bash launch.sh -t autonomy')"
        exit 1
        ;;
esac

# Attach to the 'rover_launch' tmux session (with mosh)
mosh $ROVER_USERNAME@$ROVER_IP_ADDRESS -- docker exec -it marsrover-ct tmux attach -t rover_launch

# Kill the tmux session on exit
ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "docker exec marsrover-ct tmux kill-session -t rover_launch"
