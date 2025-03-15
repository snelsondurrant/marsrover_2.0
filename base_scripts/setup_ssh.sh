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

# Check if the required tools are installed
if ! command -v sshpass &> /dev/null; then
    printError "sshpass is not installed. Please install it to proceed."
    exit 1
fi
if ! command -v mosh &> /dev/null; then
    printError "mosh is not installed. Please install it to proceed."
    exit 1
fi

ROVER_IP_ADDRESS="192.168.1.120"
ROVER_USER="marsrover"
ROVER_PWD="thekillpack"

# 1. Generate SSH key pair (if it doesn't exist)
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

# 2. Copy the public key to the rover
printInfo "Copying public key to rover..."
sshpass -p "$ROVER_PWD" ssh-copy-id -i ~/.ssh/id_rsa.pub "$ROVER_USER@$ROVER_IP_ADDRESS"
if [ $? -ne 0 ]; then
  printError "Error copying public key to rover."
  exit 1
fi

# 3. Test SSH connection without password
printInfo "Testing SSH connection without password..."
ssh "$ROVER_USER@$ROVER_IP_ADDRESS" "echo 'SSH connection successful!'"

if [ $? -ne 0 ]; then
  printError "SSH connection failed."
  exit 1
else
  echo "SSH connection successful."
fi

printInfo "SSH key setup complete."
