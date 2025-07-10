# Created by Nelson Durrant, Oct 2024
FROM ros:humble-ros-base

# Why use ROS2 Humble?
#
# The simulator is built using Gazebo Classic, which is only supported in ROS2 Iron and older.
# Unfortunately, Nav2's simple GPS navigation functions are only supported in ROS2 Iron and newer,
# but there's a simple workaround on ROS2 Humble. The ZED's container is locked to ROS2 Humble by
# compatible L4T images (it has to match the Orin), and through a lot of trial and error we've learned
# that inter-Docker container FastDDS connections really only work reliably between matching distros.
# In addition, Iron has a sooner EOL date than Humble and less community support.
#
# In summary:
# .---------.---------------.---------------.---------------.---------------.
# |         | GZ Classic?   | Nav2 GPS?     | ZED Docker?   | EOL?          |
# |-------- |---------------|---------------|---------------|---------------|
# | Humble  | yes           | workaround    | yes           | May 2027      |
# | Iron    | yes           | yes           | no            | Dec 2024      |
# | Jazzy   | no            | yes           | no            | May 2029      |
# '---------'---------------'---------------'---------------'---------------'

ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH

# Define a username and password for the new user
ARG NAME=marsrover-docker
ARG PASS=thekillpack

# Update and upgrade
RUN apt update && apt upgrade -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

# Set up a new user
RUN useradd -ms /bin/bash $NAME \
    && usermod -aG sudo $NAME \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && usermod -aG video $NAME \
    && usermod -aG dialout $NAME \
    && groupadd docker \
    && usermod -aG docker $NAME \
    && echo "$NAME:$PASS" | chpasswd
USER $NAME
WORKDIR /home/$NAME

# Install general dependencies
USER root
RUN apt update && apt install -y --no-install-recommends \
    curl \
    wget \
    git \
    vim \
    nano \
    tmux \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
USER $NAME

# Install graphical interface dependencies
USER root
RUN apt update && apt install -y --no-install-recommends \
    x11-apps \
    x11-utils \
    x11-xserver-utils \
    xauth \
    && rm -rf /var/lib/apt/lists/*
USER $NAME

# Install PlatformIO
USER root
RUN apt update && apt install -y --no-install-recommends \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*
USER $NAME

RUN curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py \
    && python3 get-platformio.py \
    && rm get-platformio.py

# Set up PlatformIO shell commands
RUN mkdir -p /usr/local/bin

USER root
RUN ln -s /home/$NAME/.platformio/penv/bin/platformio /usr/local/bin/platformio \
    && ln -s /home/$NAME/.platformio/penv/bin/pio /usr/local/bin/pio \
    && ln -s /home/$NAME/.platformio/penv/bin/piodebuggdb /usr/local/bin/piodebuggdb
USER $NAME

# Dry run PlatformIO to install the necessary dependencies
# '|| true' is used to ignore errors on a pio command failure
# TODO: There's definitely a more efficient way to do this
RUN git clone https://github.com/BYUMarsRover/marsrover_2.0.git \
    && cd /home/$NAME/marsrover_2.0/firmware/mega_pio \
    && pio pkg install || true \
    && pio run -t upload || true \
    && cd /home/$NAME/marsrover_2.0/firmware/nano_pio \
    && pio pkg install || true \
    && pio run -t upload || true \
    && rm -rf /home/$NAME/marsrover_2.0

# Set up automatic ROS2 sourcing and colorized output
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc \
    && touch ~/.hushlogin

# Add local python binaries to the PATH
ENV PATH="$PATH:/home/$NAME/.local/bin"

###############################################################
# IMPORTANT! When adding dependencies or Dockerfile commands
# below, please document your changes following this format:
#     
#     # What do these commands do? - Full Name, Month Year
#     # Why do we need them? Which ROS2 nodes require them?
#     < Add Dockerfile commands here >
#
# https://docs.docker.com/reference/dockerfile/
#
###############################################################

# Install the navigation2 and robot_localization packages - Nelson Durrant, Feb 2025
# Pre-built full navigation stack for GPS navigation and localization
USER root
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-robot-localization \
    && rm -rf /var/lib/apt/lists/*
USER $NAME

# Install graphical interface packages - Nelson Durrant, Feb 2025
# We use RViz2, RQT, and Python3-tk for GUIs
USER root
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-rqt* \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*
USER $NAME

# Install plotjuggler - Nelson Durrant, Feb 2025
# Useful for plotting and visualizing data from ROS2 bags and real-time tests
USER root
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-plotjuggler-ros \
    && rm -rf /var/lib/apt/lists/*
USER $NAME

# Install gazebo and turtlebot3 packages - Nelson Durrant, Feb 2025
# We use Gazebo Classic for the simulator and TurtleBot3 for the robot model
# These packages are not currently available for arm64
USER root
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    apt update && apt install -y --no-install-recommends \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3-gazebo \
    && rm -rf /var/lib/apt/lists/*; \
fi
USER $NAME

# Install the aruco_opencv packages - Nelson Durrant, Feb 2025
# Used for detecting ArUco markers in the simulation and real world
USER root
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-aruco-opencv \
    && rm -rf /var/lib/apt/lists/*
USER $NAME

# Install mapviz packages - Nelson Durrant, Feb 2025
# Used for visualizing the robot's path and gps location
# These packages are not currently available for arm64
USER root
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    apt update && apt install -y --no-install-recommends \
    ros-humble-mapviz \
    ros-humble-mapviz-plugins \
    ros-humble-tile-map \
    && rm -rf /var/lib/apt/lists/*; \
fi
USER $NAME

# Install mobility packages - Nelson Durrant, Feb 2025
# Used for controlling the robot's movement through a joystick or keyboard
USER root
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-teleop-twist-keyboard \
    ros-humble-topic-tools \
    && rm -rf /var/lib/apt/lists/*
USER $NAME

# Install python UTM library - Nelson Durrant, Feb 2025
# Used for converting object and aruco poses to GPS coordinates
RUN pip3 install utm

# Install turtlesim tf2 packages - Nelson Durrant, Feb 2025
# Used in the ROS2 and tf2 tutorials
USER root
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-turtle-tf2-py \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-turtlesim \
    && rm -rf /var/lib/apt/lists/*
USER $NAME

# Install ascii art packages - Nelson Durrant, Mar 2025
# Used for the Mars Rover ASCII art display on startup
RUN pip3 install --upgrade pillow asciimatics

# Install tmux docker-compose-esque launcher - Nelson Durrant, Mar 2025
# Used for launching multiple tmux windows with a single command
RUN pip3 install tmuxp

# Install pyserial - Nelson Durrant, Mar 2025
# Used for serial communication with the Arduinos
RUN pip3 install pyserial

# Install pcl_conversions - Nelson Durrant, Mar 2025
# Used for point cloud conversion in the unitree lidar package
USER root
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*
USER $NAME

# Install xacro convertor package - Nelson Durrant, Mar 2025
# Used for converting the rover's xacro files to urdf files
USER root
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-urdf-launch \
    && rm -rf /var/lib/apt/lists/*
USER $NAME

# Install zed_msgs package - Nelson Durrant, Mar 2025
# Official ROS2 interfaces for the ZED camera
USER root
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-zed-msgs \
    && rm -rf /var/lib/apt/lists/*
USER $NAME

# Install mapproxy tile caching deps - Nelson Durrant, Mar 2025
# Used for pre-caching map tiles for rover test locations
RUN pip3 install mercantile requests

# Install tools for terrain maps - Nelson Durrant, Apr 2025
# Used for loading in and parsing GeoTIFF elevation maps
USER root
RUN apt update && apt install -y --no-install-recommends \
    libgdal-dev \
    python3-rasterio \
    && rm -rf /var/lib/apt/lists/*
USER $NAME

# Install AStar planning package - Nelson Durrant, Apr 2025
# Simple A* path planning algorithm implementation for 2D maps
RUN pip3 install astar

# Install MoveIt2 package - Nelson Durrant, Jun 2025
# Open-source robotic arm manipulation platform (for use on the rover?)
USER root
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-moveit \
    && rm -rf /var/lib/apt/lists/*
USER $NAME

# Install fastmcp and uv - Nelson Durrant, July 2025
# Fast MCP server and launch system for hooking the rover controls into an LLM (experimental)
RUN pip3 install fastmcp uv
