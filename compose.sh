#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Manages the 'marsrover-ct' Docker container and the 'rover_dev' tmux session

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

script_dir=$(dirname "$(readlink -f "$0")")

# Check if the mapproxy container is already running
if [ $(docker ps | grep danielsnider/mapproxy | wc -l) -eq 0 ]; then
	printWarning "Starting the mapproxy container..."
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

		# Check if a 'rover_dev' tmux session already exists
		if [ "$(docker exec -it marsrover-ct tmux list-sessions | grep rover_dev)" == "" ]; then

			# If not, create a new 'rover_dev' tmux session
			printWarning "Creating a new 'rover_dev' tmux session..."
			envsubst < $script_dir/base_scripts/tmuxp/rover_dev.yaml > $script_dir/base_scripts/tmuxp/tmp/rover_dev.yaml
			docker exec -it marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/rover_dev.yaml
		fi
		# Attach to the 'rover_dev' tmux session
		docker exec -it marsrover-ct tmux attach -t rover_dev
    ;;
esac
