#!/bin/bash
# Created by Nelson Durrant & Braden Meyers, Feb 2025
#
# Runs automatic commands on Docker container startup
# - This script won't throw any errors, but it will crash immediately if any command fails
# - You can view the output of this script by running 'docker logs marsrover-ct'

set -e

# Are we running on Jetson Orin architecture (the rover)?
if [ "$(uname -m)" == "aarch64" ]; then

    # Start a new 'rover_startup' tmux session
    # NOTE: Using tmuxp in the minimal Docker startup env doesn't work
    tmux new-session -d -s rover_startup
    tmux set-option -g default-terminal "screen-256color"
    tmux set -g mouse on

    # Start the Fast DDS discovery server in the tmux session
    tmux send-keys -t rover_startup.0 "fastdds discovery --server-id 0" Enter # port 11811
    
    # Launch ROS 2 nodes on system startup
    tmux split-window -v
    tmux send-keys -t rover_startup.1 "export ROS_DISCOVERY_SERVER=127.0.0.1:11811" Enter
    tmux send-keys -t rover_startup.1 "source ~/rover_ws/install/setup.bash" Enter
    tmux send-keys -t rover_startup.1 "ros2 launch rover_bringup rover_startup.launch.py" Enter
fi

exec /bin/bash
