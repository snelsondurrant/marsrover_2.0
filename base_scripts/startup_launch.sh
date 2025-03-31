#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Launches startup on the rover over SSH using the 'rover_startup' tmux session

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
ROVER_USERNAME=marsrover
launch_local=false
export discovery_addr=$ROVER_IP_ADDRESS

# Check for a "-u <username>" or "-l" argument
while getopts ":u:l" opt; do
  case $opt in
    u)
      ROVER_USERNAME=$OPTARG
      ;;
    l)
      launch_local=true
      export discovery_addr=localhost
      ;;
  esac
done

# Check if the launch_local flag is set
if [ $launch_local = true ]; then
    printWarning "Launching on the local machine..."
    envsubst < tmuxp/rover_startup.yaml > tmuxp/tmp/rover_startup.yaml
    docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/rover_startup.yaml
    docker exec -it marsrover-ct tmux attach -t rover_startup
    docker exec marsrover-ct tmux kill-session -t rover_startup
    exit
fi

# Check for an SSH connection to the rover
if ! ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "echo" &> /dev/null
then
    printError "No available SSH connection to the rover"
    echo "Here's some debugging suggestions:"
    echo "  - Ensure the rover is powered on"
    echo "  - Ensure the rover is connected with a static IP address"
    echo "  - Run 'bash setup_ssh.sh' to set up SSH access"

    exit 1
fi

# Send tmux commands to the rover over SSH
printInfo "Setting up the 'rover_startup' tmux session..."
envsubst < tmuxp/rover_startup.yaml > tmuxp/tmp/rover_startup.yaml
scp tmuxp/tmp/rover_startup.yaml $ROVER_USERNAME@$ROVER_IP_ADDRESS:~/marsrover_2.0/base_scripts/tmuxp/tmp/
ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS \
	"docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/rover_startup.yaml"

# Attach to the 'rover_startup' tmux session (with mosh)
mosh $ROVER_USERNAME@$ROVER_IP_ADDRESS -- docker exec -it marsrover-ct tmux attach -t rover_startup

# Kill the tmux session on exit
ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "docker exec marsrover-ct tmux kill-session -t rover_startup"
