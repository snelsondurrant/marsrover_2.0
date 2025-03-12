#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Pull the latest Docker image and GitHub changes

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

# Are we NOT running on Jetson Orin architecture (the rover)?
if [ ! "$(uname -m)" == "aarch64" ]; then
    docker pull byuawesomerover/marsrover:latest
    git pull
else
    printError "This script is not intended to be run on the rover."
    echo "Use 'sync_docker.sh' and 'sync_git.sh' to load changes instead."
fi
