#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Builds and uploads the latest code to the board

# For some reason, I can only get this to work after a device reset
symlink="/dev/rover/onBoardMega"
device=$(readlink -f $symlink)
echo "Resetting device $device"
sysfs_path=$(udevadm info --query=path --name=$device)
echo "Sysfs path: $sysfs_path"
bus_device=$(basename $(dirname $sysfs_path))
echo "Bus device: $bus_device"
sudo usbreset $bus_device

pio run -t upload
