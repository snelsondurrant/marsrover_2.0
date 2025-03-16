#!/bin/bash
# Created by Nelson Durrant, Feb 2025
#
# The 'nav2_bringup' package doesn't install on arm64 platforms, so we have to install it explicitly
# https://github.com/ros-navigation/navigation2/issues/3766

# Check if the nav2_bringup package is already installed
if [ -d "src/nav2_bringup" ]; then
  echo "nav2_bringup package already exists, skipping installation."
  exit 0
fi

# check if arch is arm64
if [ "$(uname -m)" = "aarch64" ]; then
  # check if src/nav2_bringup does not exist
  if [ ! -d "src/nav2_bringup" ]; then
    curl -L https://api.github.com/repos/ros-planning/navigation2/tarball/1.2.2 \
      | tar xz -C src/ --wildcards "*/nav2_bringup" --strip-components=1

    # remove "turtlebot3_gazebo" dependency from nav2_bringup/package.xml since it has not arm64 build
    sed -i '/turtlebot3_gazebo/d' src/nav2_bringup/package.xml
  fi
fi