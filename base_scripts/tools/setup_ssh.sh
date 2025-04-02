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

# Check for a "-u <username>" or "-p <password>" argument
while getopts ":u:p:" opt; do
  case $opt in
    u)
      ROVER_USERNAME=$OPTARG
      ;;
    p)
      ROVER_PASSWORD=$OPTARG
      ;;
  esac
done

# Function to remove an old SSH key from known_hosts
remove_old_ssh_key() {
    local host=$1
    local port=$2

    if [[ "$port" == "22" ]]; then
        ssh-keygen -f "$HOME/.ssh/known_hosts" -R "$host" &> /dev/null
    else
        ssh-keygen -f "$HOME/.ssh/known_hosts" -R "[$host]:$port" &> /dev/null
    fi

    printInfo "Removed old SSH key for $host on port $port"
}

# 1. Remove old SSH key (if it exists)
printInfo "Removing old SSH key..."
remove_old_ssh_key "$ROVER_IP_ADDRESS" "22"

# 2. Generate SSH key pair (if it doesn't exist)
if [ ! -f ~/.ssh/id_rsa ]; then
  printInfo "Generating SSH key pair..."
  ssh-keygen -t rsa -b 4096 -N "" -f ~/.ssh/id_rsa
  if [ $? -ne 0 ]; then
    printError "Error generating SSH key pair."
    exit 1
  fi
else
  printWarning "SSH key pair already exists."
fi

# 3. Copy the public key to the rover
printInfo "Copying public key to rover..."
sshpass -p "$ROVER_PASSWORD" ssh-copy-id -i ~/.ssh/id_rsa.pub "$ROVER_USERNAME@$ROVER_IP_ADDRESS"
if [ $? -ne 0 ]; then
  printError "Error copying public key to rover."
  exit 1
fi

# 4. Test SSH connection without password
printInfo "Testing SSH connection without password..."
ssh "$ROVER_USERNAME@$ROVER_IP_ADDRESS" "echo 'SSH connection successful!'"

if [ $? -ne 0 ]; then
  printError "SSH connection failed."
  exit 1
fi

printInfo "SSH key setup complete."