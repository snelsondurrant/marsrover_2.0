#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# Builds and uploads the latest Arduino Mega code to the board

# TODO: For some reason this only works immediately after plugging the mega directly into the rover.
# It could use some research and debugging.

pio run -t upload
