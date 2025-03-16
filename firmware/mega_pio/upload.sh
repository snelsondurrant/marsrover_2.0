#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Builds and uploads the latest code to the board

# For some reason, I can only get this to work after a device reset
bus=$(lsusb | grep "Mega" | awk '{print $2}')
echo "Bus: $bus"
device=$(lsusb | grep "Mega" | awk '{print $4}')
echo "Device: $device"
usbreset $bus/$device


pio run -t upload
