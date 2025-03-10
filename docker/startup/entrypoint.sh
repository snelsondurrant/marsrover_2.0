#!/bin/bash
# Created by Nelson Durrant & Braden Meyers, Feb 2025
#
# Runs automatic commands on Docker container startup
# - This script won't throw any errors, but the container will crash immediately
# - Be very careful editing it!

# From ros_entrypoint.sh
set -e
source "/opt/ros/$ROS_DISTRO/setup.bash" # source ROS distro

# Are we running on Jetson Orin architecture (the rover)?
if [ "$(uname -m)" == "aarch64" ]; then

    # Set up the Fast DDS discovery server to run in the background
    fastdds discovery --server-id 0 & # start on port 11811

    # Start a new 'rover_startup' tmux session
    tmux new-session -d -s rover_startup
    tmux send-keys -t rover_startup.0 "clear" Enter

    # Start the Fast DDS discovery server in the tmux session
    tmux send-keys -t rover_startup.0 "fastdds discovery --server-id 0" Enter
    
    # Launch ROS 2 nodes on system startup
    # make a new pane vertically split
    tmux split-window -v
    tmux send-keys -t rover_startup.1 "source ~/mars_ws/install/setup.bash" Enter
    tmux send-keys -t rover_startup.1 "export ROS_DISCOVERY_SERVER=127.0.0.1:11811" Enter
    tmux send-keys -t rover_startup.1 "ros2 launch rover_bringup rover_startup.launch.py" Enter

    # Full color and mouse options
    tmux set-option -g default-terminal "screen-256color"
    tmux set -g mouse on
fi

# Start the SSH daemon in the Docker container
sudo /usr/sbin/sshd -D

# IMPORTANT! Keeps the container running
exec /bin/bash
