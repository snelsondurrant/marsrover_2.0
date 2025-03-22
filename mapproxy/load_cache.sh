#!/bin/bash
# Created by Nelson Durrant, Mar 2025
# 
# Preloads potential rover operation areas using MapProxy

function printError {
  	# print red
  	echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

# Check to make sure this is being run with sudo
if [ "$EUID" -ne 0 ]; then
    printError "Please run as root (sudo bash load_cache.sh)"
    exit
fi

mapproxy-seed -f mapproxy.yaml -s seed.yaml -i
