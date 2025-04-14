#!/bin/bash
# Created by Nelson Durrant, Apr 2025
# 
# Interactively resets the USB devices on the rover

echo "Do you want to reset the Arduino Mega? (y/n)"
read -r answer
if [[ $answer == "y" || $answer == "Y" ]]; then
    VENDOR=$(udevadm info --attribute-walk --path $(udevadm info --query path --name /dev/rover/onBoardMega) | grep idVendor | head -n 1 | sed 's/.*="//; s/"$//')
    PRODUCT=$(udevadm info --attribute-walk --path $(udevadm info --query path --name $1/dev/rover/onBoardMega) | grep idProduct | head -n 1 | sed 's/.*="//; s/"$//')
    sudo usbreset $VENDOR:$PRODUCT
    echo "Arduino Mega reset"
fi

echo "Do you want to reset the RTK GPS? (y/n)"
read -r answer
if [[ $answer == "y" || $answer == "Y" ]]; then
    VENDOR=(udevadm info --attribute-walk --path $(udevadm info --query path --name /dev/rover/rtk) | grep idVendor | head -n 1 | sed 's/.*="//; s/"$//')
    PRODUCT=$(udevadm info --attribute-walk --path $(udevadm info --query path --name /dev/rover/rtk) | grep idProduct | head -n 1 | sed 's/.*="//; s/"$//')
    sudo usbreset $VENDOR:$PRODUCT
    echo "RTK GPS reset"
fi
