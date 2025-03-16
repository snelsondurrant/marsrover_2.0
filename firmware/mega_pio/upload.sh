#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Builds and uploads the latest code to the board

# For some reason, I can only get this to work after a device reset
symlink="/dev/rover/onBoardMega"
# get the real path
realpath=$(readlink -f $symlink)
echo "Real path of the device: $realpath"
# reset the device
usbreset $realpath
# wait for the device to reset
sleep 2


pio run -t upload
