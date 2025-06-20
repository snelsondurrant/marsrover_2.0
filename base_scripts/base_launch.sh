#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Launches tasks on the base station using the 'base_launch' tmux session

script_dir=$(dirname "$(readlink -f "$0")")
source $script_dir/tools/base_common.sh

# Check for a "-t <task>" argument
while getopts ":t:" opt; do
  case $opt in
    t)
      task=$OPTARG
      ;;
  esac
done

# Start the Docker containers if not already running
if [ $(docker ps | grep marsrover-ct | wc -l) -eq 0 ]; then
		printWarning "Starting the marsrover-ct container..."
		cd ~/marsrover_2.0/docker && docker compose up -d
fi
if [ $(docker ps | grep mapproxy | wc -l) -eq 0 ]; then
    printWarning "Starting the mapproxy container..."
    docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy
fi

# Launch the specified task configuration over SSH
case $task in
    "autonomy")
        printInfo "Setting up the autonomy task..."
        # This envsubst allows for the use of environment variables in the tmuxp config
        envsubst < tmuxp/autonomy/base_launch.yaml > tmuxp/tmp/base_launch.yaml
        docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/base_launch.yaml
        ;;
    "servicing")
        printInfo "Setting up the servicing task..."
        # This envsubst allows for the use of environment variables in the tmuxp config
        envsubst < tmuxp/servicing/base_launch.yaml > tmuxp/tmp/base_launch.yaml
        docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/base_launch.yaml
        ;;
    "delivery")
        printInfo "Setting up the delivery task..."
        # This envsubst allows for the use of environment variables in the tmuxp config
        envsubst < tmuxp/delivery/base_launch.yaml > tmuxp/tmp/base_launch.yaml
        docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/base_launch.yaml
        ;;
    "science")
        printInfo "Setting up the science task..."
        # This envsubst allows for the use of environment variables in the tmuxp config
        envsubst < tmuxp/science/base_launch.yaml > tmuxp/tmp/base_launch.yaml
        docker exec marsrover-ct tmuxp load -d /home/marsrover-docker/.tmuxp/base_launch.yaml
        ;;
    *)
        printError "No task specified"
        echo "Specify a task using 'bash base_launch.sh -t <task>' (ex. 'bash base_launch.sh -t autonomy')"
        exit 1
        ;;
esac

# Attach to the 'base_launch' tmux session
docker exec -it marsrover-ct tmux attach -t base_launch

# Kill the tmux session on exit
docker exec marsrover-ct tmux kill-session -t base_launch
