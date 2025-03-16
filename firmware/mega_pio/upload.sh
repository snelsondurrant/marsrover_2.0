#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Builds and uploads the latest code to the board

# For some reason, I can only get this to work after a device reset
symlink="/dev/rover/onBoardMega"
# get bus and device number
bus=$(udevadm info --query=property --name=$symlink | grep DEVBUS | cut -d'=' -f2)
echo "Bus: $bus"
device=$(udevadm info --query=property --name=$symlink | grep DEVPID | cut -d'=' -f2)
echo "Device: $device"
# reset the device
usbreset $bus/$device


pio run -t upload
