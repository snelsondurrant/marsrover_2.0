#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Builds and uploads the latest Arduino Mega code to the board

# TODO: For some reason this only works on the rover immediately after plugging in the mega directly,
# and then only sometimes -- it could use some research and debugging.

VENDOR=$(udevadm info --attribute-walk --path $(udevadm info --query path --name /dev/rover/onBoardMega) | grep idVendor | head -n 1 | sed 's/.*="//; s/"$//')
PRODUCT=$(udevadm info --attribute-walk --path $(udevadm info --query path --name $1/dev/rover/onBoardMega) | grep idProduct | head -n 1 | sed 's/.*="//; s/"$//')
sudo usbreset $VENDOR:$PRODUCT

pio run -t upload --upload-port /dev/rover/onBoardMega
