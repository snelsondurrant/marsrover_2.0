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

# Launch the mapproxy container if it's not already running
if [ $(docker ps | grep danielsnider/mapproxy | wc -l) -eq 0 ]; then
	script_dir=$(dirname "$0") # the directory of this script
    docker run -p 8080:8080 -d -t -v $script_dir/mapproxy:/mapproxy danielsnider/mapproxy
fi

case $1 in
  	"down")
    	printWarning "Stopping the marsrover-ct container..."
    	docker compose -f docker/docker-compose.yaml down
    	;;
  	*)
    	printInfo "Loading the marsrover-ct container..."
    	docker compose -f docker/docker-compose.yaml up -d

    	# Are we NOT running on Jetson Orin architecture (the rover)?
    	if [ ! "$(uname -m)" == "aarch64" ]; then

			# Check if a 'rover_dev' tmux session already exists
			if [ "$(docker exec -it marsrover-ct tmux list-sessions | grep rover_dev)" == "" ]; then

				# If not, create a new 'rover_dev' tmux session
				printWarning "Creating a new 'rover_dev' tmux session..."
				docker exec -it marsrover-ct tmuxp load -d /startup/rover_dev.yaml
			fi
			# Attach to the 'rover_dev' tmux session
			docker exec -it marsrover-ct tmux attach -t rover_dev
		else

			sleep 1 # IMPORTANT! Give the Docker container a chance to start up

			# Check if the 'rover_startup' tmux session is already running
			if [ "$(docker exec -it marsrover-ct tmux list-sessions | grep rover_startup)" == "" ]; then
				printError "The 'rover_startup' tmux session is not running"
				echo "Please run 'bash compose.sh down' and then 'bash compose.sh' again"
				exit
			fi

			# Enter the 'rover_startup' tmux session on the rover
			docker exec -it marsrover-ct tmux attach -t rover_startup
		fi
    ;;
esac
