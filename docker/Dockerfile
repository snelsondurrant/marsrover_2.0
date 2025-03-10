# Created by Nelson Durrant, Oct 2024 
FROM ros:iron-ros-base
# Why use ROS 2 Iron?
# The simulator is build using Gazebo Classic, which is only supported in ROS 2 Iron and older.
# The Navigation2 package's simple GPS navigation functions are only supported in ROS 2 Iron and newer.
# So, we need to use ROS 2 Iron to run the simulator and the navigation functions at the same time.

ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH

# Define a username and password for the new user
ARG NAME=marsrover-docker
ARG PASS=marsrover

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

# Set up the direct Docker ssh connection
USER root
RUN apt update && apt install -y openssh-server
RUN mkdir -p /var/run/sshd
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config
RUN sed -i 's/PasswordAuthentication no/PasswordAuthentication yes/' /etc/ssh/sshd_config
RUN sed -i 's/\(^Port\)/#\1/' /etc/ssh/sshd_config && echo Port 2233 >> /etc/ssh/sshd_config
EXPOSE 2233
USER $NAME

# Install general dependencies
USER root
RUN apt update && apt install -y \
    curl \
    wget \
    git \
    vim \
    nano \
    tmux \
    docker.io \
    python3-pip
USER $NAME

# Install graphical interface dependencies
USER root
RUN apt update && apt install -y \
    x11-apps \
    x11-utils \
    x11-xserver-utils \
    xauth 
USER $NAME

# Install PlatformIO
USER root
RUN apt install -y python3-venv
USER $NAME

RUN curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
RUN python3 get-platformio.py
RUN rm get-platformio.py

# Set up PlatformIO shell commands
RUN mkdir -p /usr/local/bin

USER root
RUN ln -s /home/$NAME/.platformio/penv/bin/platformio /usr/local/bin/platformio
RUN ln -s /home/$NAME/.platformio/penv/bin/pio /usr/local/bin/pio
RUN ln -s /home/$NAME/.platformio/penv/bin/piodebuggdb /usr/local/bin/piodebuggdb
USER $NAME

# Set up automatic ROS 2 sourcing and colorized output
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
RUN touch ~/.hushlogin

# Add local python binaries to the PATH
ENV PATH="$PATH:/home/$NAME/.local/bin"

###############################################################
# IMPORTANT! When adding dependencies or Dockerfile commands
# below, please document your changes following this format:
#     
#     # What do these commands do? - Full Name, Month Year
#     # Why do we need them? Which ROS 2 nodes require them?
#     < Add Dockerfile commands here >
#
###############################################################

# Install the navigation2 and robot_localization packages - Nelson Durrant, Feb 2024
# Pre-built full navigation stack for GPS navigation and localization
USER root
RUN apt update && apt install -y \
    ros-iron-navigation2 \
    ros-iron-robot-localization
USER $NAME

# Install graphical interface packages - Nelson Durrant, Feb 2024
# We use RViz2, RQT, and Python3-tk for GUIs
USER root
RUN apt update && apt install -y \
    ros-iron-rviz2 \
    ros-iron-rqt* \
    python3-tk
USER $NAME

# Install plotjuggler - Nelson Durrant, Feb 2024
# Useful for plotting and visualizing data from ROS 2 bags
USER root
RUN apt update && apt install -y ros-iron-plotjuggler-ros
USER $NAME

# Install gazebo and turtlebot3 packages - Nelson Durrant, Feb 2024
# We use Gazebo Classic for the simulator and TurtleBot3 for the robot model
# For some reason, these packages are not available for arm64
USER root
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    apt update && apt install -y \
    ros-iron-nav2-bringup \
    ros-iron-turtlebot3-gazebo; \
fi
USER $NAME

# Install the aruco_opencv packages - Nelson Durrant, Feb 2024
# Used for detecting ArUco markers in the simulation and real world
USER root
RUN apt update && apt install -y \
    ros-iron-aruco-opencv \
    ros-iron-usb-cam
USER $NAME

# Install mapviz packages - Nelson Durrant, Feb 2024
# Used for visualizing the robot's path and gps location
# For some reason, these packages are not available for arm64
USER root
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    apt update && apt install -y \
    ros-iron-mapviz \
    ros-iron-mapviz-plugins \
    ros-iron-tile-map; \
fi
USER $NAME

# Install mobility packages - Nelson Durrant, Feb 2024
# Used for controlling the robot's movement through a joystick or keyboard
USER root
RUN apt update && apt install -y \
    ros-iron-joy \
    ros-iron-teleop-twist-joy \
    ros-iron-teleop-twist-keyboard \
    ros-iron-topic-tools
USER $NAME

# Install zed interface packages - Nelson Durrant, Feb 2024
# Necessary message package to listen to topics from the ZED camera
USER root
RUN apt update && apt install -y \
    ros-iron-vision-msgs \
    ros-iron-vision-msgs-rviz-plugins
USER $NAME

# Install python UTM library - Nelson Durrant, Feb 2024
# Used for converting object and aruco poses to GPS coordinates
RUN pip3 install utm

# Install turtlesim tf2 packages - Nelson Durrant, Feb 2024
# Used in the ROS 2 and tf2 tutorials
USER root
RUN apt update && apt install -y \
    ros-iron-turtle-tf2-py \
    ros-iron-tf2-ros \
    ros-iron-tf2-tools \
    ros-iron-turtlesim
USER $NAME

# Install ascii art packages - Nelson Durrant, Mar 2024
# Used for the Mars Rover ASCII art display on startup
RUN pip3 install --upgrade pillow asciimatics

# Install tmux docker-compose-esque launcher - Nelson Durrant, Mar 2024
# Used for launching multiple tmux windows with a single command
RUN pip3 install tmuxp

# Install matplotlib - Nelson Durrant, Mar 2024
# Used for plotting calculated path planning routes
RUN pip3 install matplotlib

# Install pyserial - Nelson Durrant, Mar 2024
# Used for serial communication with the Arduino
RUN pip3 install pyserial
