# Created by Braden Meyers, MAR 2025 

# Pull from Sterolabs image
# Sterolabs docker image repository: https://hub.docker.com/r/stereolabs/zed/tags
# Make sure the jetpack version matches the jetpack version on the computer!!!
FROM stereolabs/zed:4.2-tools-devel-jetson-jp5.1.2

ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH

# Define a username and password for the new user
ARG NAME=zed-docker
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


# Install ROS 2
USER root
RUN apt update && apt install -y \
    build-essential \
    git \
    locales \
    software-properties-common
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
USER $NAME
RUN export LANG=en_US.UTF-8

USER root
RUN add-apt-repository universe
RUN apt update
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt upgrade -y
RUN apt install -y ros-foxy-ros-base
RUN apt install ros-dev-tools -y
USER $NAME

USER root
# PERMISSION FIX ON PIP INSTALLATIONS
RUN chown -R zed-docker:zed /usr/local/lib/python3.8/dist-packages/

USER $NAME

# source foxy in the bashrc

# Install the aruco_opencv packages - Nelson Durrant, Feb 2024
# Used for detecting ArUco markers in the simulation and real world
USER root
RUN apt install -y \
    ros-foxy-cv-bridge \
    ros-foxy-image-transport 
USER $NAME



# Message type for publishing object detections
USER root
RUN apt install -y ros-foxy-vision-msgs ros-foxy-geographic-msgs ros-foxy-nmea-msgs

# Give permissions to the zed installation
#TODO move this up
RUN usermod -aG zed $NAME
RUN rosdep init


USER $NAME
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

