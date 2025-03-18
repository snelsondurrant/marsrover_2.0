#!/bin/bash
# Created by Nelson Durrant, Mar 2025
#
# Reloads and triggers the udev rules in this folder

script_dir=$(dirname "$0") # the directory of this script
sudo cp $script_dir/*.rules /etc/udev/rules.d/

sudo udevadm control --reload-rules
sudo udevadm trigger
