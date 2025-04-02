#!/bin/bash
# Created by Nelson Durrant, Mar 2025
#
# Pulls the the latest Docker image onto the rover

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

# Check for a "-u <username>" argument
while getopts ":u:" opt; do
  case $opt in
    u)
      ROVER_USERNAME=$OPTARG
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
    echo "  - Run 'cd tools && bash setup_ssh.sh' to set up SSH access"

    exit 1
fi

# Check the architecture of the rover
rover_arch=$(ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "uname -m")
if [[ "$rover_arch" == "aarch64" ]]; then
    rover_id="linux/arm64"
else
    rover_id="linux/amd64"
fi

# Pull the platform-specific version of the Docker image
printInfo "Pulling the $rover_id version of the Docker image..."
docker pull --platform $rover_id byuawesomerover/marsrover_2.0:latest
docker image list

printInfo "Saving the $rover_id Docker image to a zip file for transfer (this takes a while)..."
docker save byuawesomerover/marsrover_2.0:latest | gzip > marsrover_2.0.tar.gz

printInfo "Sending the Docker image to the rover..."
scp marsrover_2.0.tar.gz $ROVER_USERNAME@$ROVER_IP_ADDRESS:~/marsrover_2.0/docker
rm marsrover_2.0.tar.gz

# Pull the amd64 version of the Docker image
# (this will override the version we just pulled)
printInfo "Pulling the linux/amd64 version of the Docker image..."
docker pull --platform linux/amd64 byuawesomerover/marsrover_2.0:latest
yes | docker image prune
docker image list

# Send tmux commands to the rover over SSH
printInfo "Setting up the 'docker_sync' tmux session..."
envsubst < tmuxp/docker_sync.yaml > tmuxp/tmp/docker_sync.yaml
scp tmuxp/tmp/docker_sync.yaml $ROVER_USERNAME@$ROVER_IP_ADDRESS:~/marsrover_2.0/base_scripts/tmuxp/tmp/
ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS \
    "export PATH='$PATH:/home/$ROVER_USERNAME/.local/bin'; \
    tmuxp load -d /home/$ROVER_USERNAME/marsrover_2.0/base_scripts/tmuxp/tmp/docker_sync.yaml"

# Attach to the 'docker_sync' tmux session to view the output (using mosh)
mosh $ROVER_USERNAME@$ROVER_IP_ADDRESS -- tmux attach -t docker_sync

# Kill the tmux session on exit
ssh $ROVER_USERNAME@$ROVER_IP_ADDRESS "tmux kill-session -t docker_sync"
