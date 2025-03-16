#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Builds and uploads the latest code to the board

# For some reason, I can only get this to work after a device reset
symlink="/dev/rover/onBoardMega"
device=$(readlink -f $symlink)
echo "Resetting device $device"
# Use lsusb to find bus and device numbers
bus_device=$(lsusb | grep "$(basename $device)" | awk '{print $2"-"$4}' | sed 's/://g')
echo "Bus-Device: $bus_device"

sudo usbreset $bus_device

pio run -t upload
