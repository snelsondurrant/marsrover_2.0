#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Builds and uploads the latest code to the board

# For some reason, I can only get this to work after a device reset
symlink="/dev/rover/onBoardMega"
device=$(readlink -f $symlink)
sysfs_path=$(udevadm info --query=path --name=$device)
bus_device=$(basename $(dirname $sysfs_path))
sudo usbreset $bus_device

pio run -t upload
