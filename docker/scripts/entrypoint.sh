#!/bin/bash
# Created by Nelson Durrant & Braden Meyers, Feb 2025
#
# Runs automatic commands on Docker container startup
# - This script won't throw any errors, but the container will crash immediately
# - Be very careful editing it!

# Are we running on Jetson Orin architecture (the rover)?
if [ "$(uname -m)" == "aarch64" ]; then

    # Start a new 'rover_runtime' tmux session
    tmux new-session -d -s rover_runtime
    tmux send-keys -t rover_runtime.0 "clear" Enter

    # Make a new window for Fast DDS discovery
    tmux new-window -t rover_runtime
    tmux send-keys -t rover_runtime.1 "clear" Enter
    tmux send-keys -t rover_runtime.1 "fastdds discovery --server-id 0" Enter # start on port 11811
    
    # Launch ROS 2 nodes on system startup
    tmux send-keys -t rover_runtime.0 "source ~/mars_ws/install/setup.bash" Enter
    tmux send-keys -t rover_runtime.0 "export ROS_DISCOVERY_SERVER=127.0.0.1:11811" Enter
    tmux send-keys -t rover_runtime.0 "ros2 launch mobility rover_xbox_launch.py" Enter

    # Full color and mouse options
    tmux set-option -g default-terminal "screen-256color"
    tmux set -g mouse on
fi

# Start the SSH daemon in the Docker container
sudo /usr/sbin/sshd -D

# IMPORTANT! Keeps the container running
exec "$@"
