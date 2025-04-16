#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Builds and uploads the latest Arduino Nano code to the board

VENDOR=$(udevadm info --attribute-walk --path $(udevadm info --query path --name /dev/rover/peripheralsBoard) | grep idVendor | head -n 1 | sed 's/.*="//; s/"$//')
PRODUCT=$(udevadm info --attribute-walk --path $(udevadm info --query path --name /dev/rover/peripheralsBoard) | grep idProduct | head -n 1 | sed 's/.*="//; s/"$//')
sudo usbreset $VENDOR:$PRODUCT

pio run -t upload --upload-port /dev/rover/peripheralsBoard
