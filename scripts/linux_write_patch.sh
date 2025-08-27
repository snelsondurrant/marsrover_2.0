#!/bin/bash
# Created by Nelson Durrant, Aug 2025
# 
# Quick patch for initial write permission errors on Linux computers

sudo chmod -R a+w /home/marsrover-docker/rover_ws
sudo chmod -R a+w /home/marsrover-docker/firmware
sudo chmod -R a+w /home/marsrover-docker/scripts
sudo chmod -R a+w /home/marsrover-docker/tutorial_ws
sudo chmod -R a+w /home/marsrover-docker/.tmuxp
sudo chmod -R a+w /startup
