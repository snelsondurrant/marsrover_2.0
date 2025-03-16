#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Builds and uploads the latest code to the board

# TODO: For some reason, I can only get this to work after a device reset
# bus=$(lsusb | grep "Mega" | awk '{print $2}')
# device=$(lsusb | grep "Mega" | awk '{print $4}')
# usbreset $bus/$device

pio run -t upload
