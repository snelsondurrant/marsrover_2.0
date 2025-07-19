#!/bin/bash
# Created by Nelson Durrant, July 2025
#
# Script to build the GTSAM localization testing environment

script_dir=$(dirname "$(readlink -f "$0")")
docker build -t byuawesomerover/marsrover_2.0:gtsam $script_dir