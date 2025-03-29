#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Launches the zed camera over SSH using the 'zed_launch' tmux session

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

# Check for a "-a <ip_address>", "-u <username>", or "-l" argument
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
    envsubst < tmuxp/autonomy/zed_launch.yaml > tmuxp/tmp/zed_launch.yaml
    docker exec zed-ct tmuxp load -d /home/marsrover-zed/.tmuxp/zed_launch.yaml
    docker exec -it zed-ct tmux attach -t zed_launch
    docker exec zed-ct tmux kill-session -t zed_launch
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
printInfo "Setting up the 'zed_launch' tmux session..."
envsubst < tmuxp/autonomy/zed_launch.yaml > tmuxp/tmp/zed_launch.yaml
scp tmuxp/tmp/zed_launch.yaml $ROVER_USERNAME@$ROVER_IP_ADDRESS:~/marsrover/base_scripts/tmuxp/tmp/
ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS \
	"docker exec zed-ct tmuxp load -d /home/marsrover-zed/.tmuxp/zed_launch.yaml"

# Attach to the 'zed_launch' tmux session (with mosh)
mosh $ROVER_USERNAME@$ROVER_IP_ADDRESS -- docker exec -it zed-ct tmux attach -t zed_launch

# Kill the tmux session on exit
ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "docker exec zed-ct tmux kill-session -t zed_launch"
