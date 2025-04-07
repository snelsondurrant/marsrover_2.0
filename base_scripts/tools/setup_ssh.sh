#!/bin/bash
# Created by Braden Meyers, Mar 2025
#
# Set up a SSH key for passwordless access to the Mars Rover

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
ROVER_PASSWORD=thekillpack

# Check for a "-u <username>" argument
while getopts ":u:p:" opt; do
  case $opt in
    u)
      ROVER_USERNAME=$OPTARG
      ;;
    p)
      ROVER_PWD=$OPTARG
      ;;
  esac
done

# Function to remove an old SSH key from known_hosts
# This prevents SSH errors if the host's key has changed.
remove_old_ssh_key() {
    local host=$1
    local port=$2

    if [[ "$port" == "22" ]]; then
        # Remove without specifying port notation for default SSH port
        ssh-keygen -f "$HOME/.ssh/known_hosts" -R "$host" &> /dev/null
    else
        # Remove using port notation for non-default ports
        ssh-keygen -f "$HOME/.ssh/known_hosts" -R "[$host]:$port" &> /dev/null
    fi

    printInfo "Removed old SSH key for $host on port $port"
}

# Check if ssh pass is installed
if ! command -v sshpass &> /dev/null; then
    printWarning "Please install 'sshpass' using (sudo apt install sshpass)"
    ROVER_CONNECT="ssh"
else
    ROVER_CONNECT="sshpass -p $ROVER_PWD ssh"
fi

# Remove old SSH key (if it exists) to prevent key mismatch issues
remove_old_ssh_key "$ROVER_IP_ADDRESS" "22"

# Test the SSH connection to accept the new key if prompted
$ROVER_CONNECT -o StrictHostKeyChecking=accept-new $ROVER_USERNAME@$ROVER_IP_ADDRESS "echo" &> /dev/null

# Copy the SSH key to the rover (physical machine)
ssh-copy-id marsrover@$ROVER_IP_ADDRESS

printInfo "SSH key successfully set up on the rover"
