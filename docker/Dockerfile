# Created by Nelson Durrant, Oct 2024
FROM ros:humble-ros-base

# Why use ROS 2 Humble?
#
# The simulator is built using Gazebo Classic, which is only supported in ROS 2 Iron and older.
# Nav2's simple GPS navigation functions are only supported in ROS 2 Iron and newer, but
# there's a simple workaround on ROS 2 Humble. The ZED's container is locked to ROS 2 Humble by
# compatible L4T images (it has to match the Orin), and through a lot of trial and error we've learned
# that inter-Docker container FastDDS connections really only work reliably between matching distros.
# In addition, Iron has a sooner EOL date than Humble and less community support.

ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH

# Define a username and password for the new user
ARG NAME=marsrover-docker
ARG PASS=thekillpack

# Update and upgrade
RUN apt update && apt upgrade -y

# Set up a new user
RUN useradd -ms /bin/bash $NAME
RUN usermod -aG sudo $NAME
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN usermod -aG video $NAME
RUN usermod -aG dialout $NAME
RUN groupadd docker
RUN usermod -aG docker $NAME
RUN echo "$NAME:$PASS" | chpasswd
USER $NAME
WORKDIR /home/$NAME

# Install general dependencies
USER root
RUN apt update && apt install -y \
    curl \
    wget \
    git \
    vim \
    nano \
    tmux \
    python3-pip
USER $NAME

# # Install graphical interface dependencies
# USER root
# RUN apt update && apt install -y \
#     x11-apps \
#     x11-utils \
#     x11-xserver-utils \
#     xauth 
# USER $NAME

# # Install PlatformIO
# USER root
# RUN apt install -y python3-venv
# USER $NAME

# RUN curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
# RUN python3 get-platformio.py
# RUN rm get-platformio.py

# # Set up PlatformIO shell commands
# RUN mkdir -p /usr/local/bin

# USER root
# RUN ln -s /home/$NAME/.platformio/penv/bin/platformio /usr/local/bin/platformio
# RUN ln -s /home/$NAME/.platformio/penv/bin/pio /usr/local/bin/pio
# RUN ln -s /home/$NAME/.platformio/penv/bin/piodebuggdb /usr/local/bin/piodebuggdb
# USER $NAME

# # Dry run PlatformIO to install the necessary dependencies
# RUN git clone https://github.com/BYUMarsRover/marsrover_2.0.git
# # '|| true' is used to ignore errors on a pio command failure
# WORKDIR /home/$NAME/marsrover_2.0/firmware/mega_pio
# RUN pio pkg install || true
# RUN pio run -t upload || true
# WORKDIR /home/$NAME/marsrover_2.0/firmware/nano_pio
# RUN pio pkg install || true
# RUN pio run -t upload || true
# WORKDIR /home/$NAME
# RUN rm -rf /home/$NAME/marsrover_2.0

# # Set up automatic ROS 2 sourcing and colorized output
# RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
# RUN touch ~/.hushlogin

# # Add local python binaries to the PATH
# ENV PATH="$PATH:/home/$NAME/.local/bin"

# ###############################################################
# # IMPORTANT! When adding dependencies or Dockerfile commands
# # below, please document your changes following this format:
# #     
# #     # What do these commands do? - Full Name, Month Year
# #     # Why do we need them? Which ROS 2 nodes require them?
# #     < Add Dockerfile commands here >
# #
# # https://docs.docker.com/reference/dockerfile/
# #
# ###############################################################

# # Install the navigation2 and robot_localization packages - Nelson Durrant, Feb 2025
# # Pre-built full navigation stack for GPS navigation and localization
# USER root
# RUN apt update && apt install -y \
#     ros-humble-navigation2 \
#     ros-humble-robot-localization
# USER $NAME

# # Install graphical interface packages - Nelson Durrant, Feb 2025
# # We use RViz2, RQT, and Python3-tk for GUIs
# USER root
# RUN apt update && apt install -y \
#     ros-humble-rviz2 \
#     ros-humble-rqt* \
#     python3-tk
# USER $NAME

# # Install plotjuggler - Nelson Durrant, Feb 2025
# # Useful for plotting and visualizing data from ROS 2 bags
# USER root
# RUN apt update && apt install -y ros-humble-plotjuggler-ros
# USER $NAME

# # Install gazebo and turtlebot3 packages - Nelson Durrant, Feb 2025
# # We use Gazebo Classic for the simulator and TurtleBot3 for the robot model
# # For some reason, these packages are not available for arm64
# USER root
# RUN if [ "$TARGETARCH" = "amd64" ]; then \
#     apt update && apt install -y \
#     ros-humble-nav2-bringup \
#     ros-humble-turtlebot3-gazebo; \
# fi
# USER $NAME

# # Install the aruco_opencv packages - Nelson Durrant, Feb 2025
# # Used for detecting ArUco markers in the simulation and real world
# USER root
# RUN apt update && apt install -y ros-humble-aruco-opencv
# USER $NAME

# # Install mapviz packages - Nelson Durrant, Feb 2025
# # Used for visualizing the robot's path and gps location
# # For some reason, these packages are not available for arm64
# USER root
# RUN if [ "$TARGETARCH" = "amd64" ]; then \
#     apt update && apt install -y \
#     ros-humble-mapviz \
#     ros-humble-mapviz-plugins \
#     ros-humble-tile-map; \
# fi
# USER $NAME

# # Install mobility packages - Nelson Durrant, Feb 2025
# # Used for controlling the robot's movement through a joystick or keyboard
# USER root
# RUN apt update && apt install -y \
#     ros-humble-joy \
#     ros-humble-teleop-twist-joy \
#     ros-humble-teleop-twist-keyboard \
#     ros-humble-topic-tools
# USER $NAME

# # Install python UTM library - Nelson Durrant, Feb 2025
# # Used for converting object and aruco poses to GPS coordinates
# RUN pip3 install utm

# # Install turtlesim tf2 packages - Nelson Durrant, Feb 2025
# # Used in the ROS 2 and tf2 tutorials
# USER root
# RUN apt update && apt install -y \
#     ros-humble-turtle-tf2-py \
#     ros-humble-tf2-ros \
#     ros-humble-tf2-tools \
#     ros-humble-turtlesim
# USER $NAME

# # Install ascii art packages - Nelson Durrant, Mar 2025
# # Used for the Mars Rover ASCII art display on startup
# RUN pip3 install --upgrade pillow asciimatics

# # Install tmux docker-compose-esque launcher - Nelson Durrant, Mar 2025
# # Used for launching multiple tmux windows with a single command
# RUN pip3 install tmuxp

# # Install matplotlib - Nelson Durrant, Mar 2025
# # Used for plotting calculated path planning routes
# RUN pip3 install matplotlib

# # Install pyserial - Nelson Durrant, Mar 2025
# # Used for serial communication with the Arduinos
# RUN pip3 install pyserial

# # Install pcl_conversions - Nelson Durrant, Mar 2025
# # Used for point cloud conversion in the unitree lidar package
# USER root
# RUN apt update && apt install -y ros-humble-pcl-conversions
# USER $NAME

# # Install xacro convertor package - Nelson Durrant, Mar 2025
# # Used for converting the rover's xacro files to urdf files
# USER root
# RUN apt update && apt install -y \
#     ros-humble-xacro \
#     ros-humble-joint-state-publisher-gui \
#     ros-humble-urdf-launch
# USER $NAME

# # Install zed_msgs package - Nelson Durrant, Mar 2025
# # Official ROS 2 interfaces for the ZED camera
# USER root
# RUN apt update && apt install -y ros-humble-zed-msgs
# USER $NAME

# # Install IMU filter package - Nelson Durrant, Mar 2025
# # Used to combine the ZED's IMU data with its magnetometer data
# USER root
# RUN apt update && apt install -y ros-humble-imu-filter-madgwick
# USER $NAME

# # Install mapproxy tile caching script deps - Nelson Durrant, Mar 2025
# RUN pip3 install mercantile
# RUN pip3 install requests

# # Install tf transformations package - Braden Meyers, Mar 2025
# # Used to calibrate the ZED and lidar positions relative to the mount frame
# USER root
# RUN apt update && apt install -y ros-humble-tf-transformations
# USER $NAME 

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    gnupg \
    software-properties-common \
    wget && \
    # Add the ubuntugis PPA for newer GDAL versions
    add-apt-repository ppa:ubuntugis/unstable && \
    # Update package list again to recognize the PPA
    apt-get update && \
    # Install GDAL development libraries and command-line tools from the PPA
    # This should pull in a recent version like 3.9.x if available
    apt-get install -y --no-install-recommends \
      libgdal-dev \
      gdal-bin
USER $NAME

# Install tools for terrain path planning - Nelson Durrant, Apr 2025
RUN if [ ! "$TARGETARCH" = "amd64" ]; then \
    pip3 install rasterio && \
    pip3 install astar; \
fi
