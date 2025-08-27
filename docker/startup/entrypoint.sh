#!/bin/bash
# Created by Nelson Durrant & Braden Meyers, Feb 2025
#
# Runs automatic commands on Docker container startup
# - This script won't throw any errors, but it will crash immediately if any command fails
# - You can view the output of this script by running 'docker logs marsrover-ct'

set -e

# Fix initial write permission errors on Linux computers
sudo chmod -R a+w /home/marsrover-docker/rover_ws
sudo chmod -R a+w /home/marsrover-docker/firmware
sudo chmod -R a+w /home/marsrover-docker/scripts
sudo chmod -R a+w /home/marsrover-docker/tutorial_ws
sudo chmod -R a+w /home/marsrover-docker/.tmuxp
sudo chmod -R a+w /startup
echo "Successfully applied write permissions patch."

# Any tasks that need to be run on startup should be added here

exec /bin/bash
