# Created by Nelson Durrant, Oct 2024
FROM ros:jazzy-ros-base
ARG DEBIAN_FRONTEND=noninteractive
ARG TARGETARCH

# Define a username for the new user
ARG NAME=marsrover

# Update and upgrade
RUN apt update && apt upgrade -y

# Set up the direct Docker ssh connection
RUN sudo apt update && apt install -y openssh-server
RUN mkdir -p /var/run/sshd
RUN sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config
RUN sed -i 's/PasswordAuthentication no/PasswordAuthentication yes/' /etc/ssh/sshd_config
RUN sed -i 's/\(^Port\)/#\1/' /etc/ssh/sshd_config && echo Port 2233 >> /etc/ssh/sshd_config
EXPOSE 2233

# Install general dependencies
RUN apt update && apt install -y \
    sudo \
    curl \
    vim \
    tmux \
    docker.io \
    python3-pip

# Install graphical interface dependencies
RUN apt update && apt install -y \
    x11-apps \
    x11-utils \
    x11-xserver-utils \
    xauth

# Install the Nav2 package
RUN apt update && apt install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup
RUN apt update && apt install -y ros-jazzy-nav2-minimal-tb*

# Install mapviz for the GPS demo
RUN apt update && apt install -y \
    ros-jazzy-robot-localization

# Mapviz doesn't work on arm64
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    apt update && apt install -y \
    ros-jazzy-mapviz \
    ros-jazzy-mapviz-plugins \
    ros-jazzy-tile-map; \
fi

# Set up a new user
RUN useradd -ms /bin/bash $NAME
RUN usermod -aG sudo $NAME
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN usermod -aG video $NAME
RUN usermod -aG dialout $NAME
RUN echo "$NAME:$NAME" | chpasswd
USER $NAME
WORKDIR /home/$NAME

# Set up automatic ROS 2 sourcing and colorized output
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /home/$NAME/ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
RUN touch ~/.hushlogin

###############################################################
# IMPORTANT! When adding dependencies or Dockerfile commands
# below, please document your changes following this format:
#     
#     # What do these commands do? - Name, Month Year
#     # Why do we need them? Which ROS 2 nodes require them?
#     < Add Dockerfile commands here >
#
###############################################################

# Install domain bridge package - Nelson Durrant, Feb 2025
# Necessary dependency for running the domain bridge scripts
USER root
RUN apt update && apt install -y ros-jazzy-domain-bridge
USER $NAME
