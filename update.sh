#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Pull the latest Docker image and GitHub changes

docker pull byuawesomerover/marsrover_2.0:latest
current_branch=$(git branch --show-current)
git pull origin $current_branch # specify upstream for the rover's git setup
