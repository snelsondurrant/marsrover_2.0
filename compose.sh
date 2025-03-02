#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Manages Docker containers and the 'rover_dev' tmux session

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

# Match this username to the one defined in the Dockerfile
export NAME=marsrover-docker

case $1 in
  	"down")
    	printWarning "Stopping the marsrover-ct container..."
    	docker compose -f docker/docker-compose.yaml down
    	;;
  	*)
    	printInfo "Loading the marsrover-ct container..."
    	docker compose -f docker/docker-compose.yaml up -d

    	# Are we running on Jetson Orin architecture (the rover)?
    	if [ ! "$(uname -m)" == "aarch64" ]; then

			# Check if a 'rover_dev' tmux session already exists
			if [ "$(docker exec -it marsrover-ct tmux list-sessions | grep rover_dev)" == "" ]; then

				# If not, create a new 'rover_dev' tmux session
				printWarning "Creating a new tmux session..."
				docker exec -it marsrover-ct tmux new-session -d -s rover_dev
				docker exec -it marsrover-ct tmux select-pane -t 0 -T rover_dev
				docker exec -it marsrover-ct tmux send-keys "clear && cat /startup/introduction.txt" Enter

				# Full color and mouse options
				docker exec -it marsrover-ct tmux set-option -g default-terminal "screen-256color"
				docker exec -it marsrover-ct tmux set -g mouse on
			fi
			# Attach to the 'rover_dev' tmux session
			docker exec -it marsrover-ct tmux attach -t rover_dev
		else

			sleep 1 # IMPORTANT! Give the Docker container a chance to start up

			# Enter the 'rover_runtime' tmux session on the rover
			docker exec -it marsrover-ct tmux attach -t rover_runtime
		fi
    ;;
esac
