#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Builds and uploads the latest Arduino Mega code to the board

# TODO: For some reason this only works on the rover immediately after plugging in the mega directly,
# and then only sometimes -- it could use some research and debugging.

pio run -t upload
