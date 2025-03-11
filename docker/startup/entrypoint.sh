#!/bin/bash
# Created by Nelson Durrant & Braden Meyers, Feb 2025
#
# Runs automatic commands on Docker container startup
# - This script won't throw any errors, but the container will crash immediately
# - Be very careful editing it!

# Are we running on Jetson Orin architecture (the rover)?
if [ "$(uname -m)" == "aarch64" ]; then
    tmuxp load -d /startup/workspaces/rover_startup.yaml
fi

# IMPORTANT! Keeps the container running
exec $@
