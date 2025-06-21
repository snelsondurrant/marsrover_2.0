# Created by Nelson Durrant & Braden Meyers, Mar 2025
ARG L4T_VERSION=l4t-r36.3.0
ARG IMAGE_NAME=dustynv/ros:humble-ros-base-${L4T_VERSION}

FROM ${IMAGE_NAME}

ARG ZED_SDK_MAJOR=4
ARG ZED_SDK_MINOR=2
ARG ZED_SDK_PATCH=3
ARG L4T_MAJOR=36
ARG L4T_MINOR=3

# ROS2 distribution
ARG ROS2_DIST=humble

# ZED ROS2 Wrapper dependencies version
ARG XACRO_VERSION=2.0.8
ARG DIAGNOSTICS_VERSION=4.0.0
ARG AMENT_LINT_VERSION=0.12.11
ARG ROBOT_LOCALIZATION_VERSION=3.5.3
ARG ZED_MSGS_VERSION=4.2.2
ARG NMEA_MSGS_VERSION=2.0.0
ARG ANGLES_VERSION=1.15.0
ARG GEOGRAPHIC_INFO_VERSION=1.0.6
ARG POINTCLOUD_TRANSPORT_VERSION=1.0.18
ARG POINTCLOUD_TRANSPORT_PLUGINS_VERSION=1.0.11

# Define a username and password for the new user
ARG NAME=marsrover-zed
ARG PASS=thekillpack

# Set up a new user
RUN useradd -ms /bin/bash $NAME
RUN usermod -aG sudo $NAME
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN usermod -aG video $NAME
RUN usermod -aG dialout $NAME
RUN groupadd docker
RUN usermod -aG docker $NAME
RUN echo "$NAME:$PASS" | chpasswd

ENV DEBIAN_FRONTEND=noninteractive

# ZED SDK link
ENV ZED_SDK_URL="https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/ZED_SDK_Tegra_L4T${L4T_MAJOR}.${L4T_MINOR}_v${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}.${ZED_SDK_PATCH}.zstd.run"

# Check that this SDK exists
RUN if [ "$(curl -I "${ZED_SDK_URL}" -o /dev/null -s -w '%{http_code}\n' | head -n 1)" = "200" ]; then \
        echo "The URL points to something."; \
    else \
        echo "The URL does not point to a .run file or the file does not exist."; \
        exit 1; \
    fi

# Disable apt-get warnings
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 42D5A192B819C5DA || true && \
  apt-get update || true && apt-get install -y --no-install-recommends apt-utils dialog && \
  rm -rf /var/lib/apt/lists/*

ENV TZ=Europe/Paris

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \ 
  apt-get update && \
  apt-get install --yes lsb-release wget less udev sudo build-essential cmake python3 python3-dev python3-pip python3-wheel git jq libpq-dev zstd usbutils && \    
  rm -rf /var/lib/apt/lists/*

RUN echo "# R${L4T_MAJOR} (release), REVISION: ${L4T_MINOR}" > /etc/nv_tegra_release && \
  apt-get update -y || true && \
  apt-get install -y --no-install-recommends zstd wget less cmake curl gnupg2 \
  build-essential python3 python3-pip python3-dev python3-setuptools libusb-1.0-0-dev \ 
  libgeographic-dev libdraco-dev zlib1g-dev -y && \
  pip install protobuf && \
  wget -q --no-check-certificate -O ZED_SDK_Linux_JP.run \
  ${ZED_SDK_URL} && \
  chmod +x ZED_SDK_Linux_JP.run ; ./ZED_SDK_Linux_JP.run silent skip_tools && \
  rm -rf /usr/local/zed/resources/* && \
  rm -rf ZED_SDK_Linux_JP.run && \
  rm -rf /var/lib/apt/lists/*

# Install the ZED ROS2 Wrapper
ENV ROS_DISTRO=${ROS2_DIST}

# Install missing dependencies from the basic ZED ROS2 container
RUN apt update && apt install -y \
    ros-humble-ros-base \
    ros-humble-zed-msgs \
    ros-humble-xacro \
    ros-humble-ament-lint \
    ros-humble-point-cloud-transport \
    ros-humble-cob-srvs \
    ros-humble-geographic-msgs \
    ros-humble-unique-identifier-msgs \
    ros-humble-rosidl-default-generators \
    ros-humble-nmea-msgs \
    ros-humble-robot-localization \
    ros-humble-angles \  
    ros-humble-stereo-msgs \
    ros-humble-image-transport \     
    ros-humble-visualization-msgs
    
# Install cython
RUN python3 -m pip install --upgrade cython

# Install missing build tools from the basic ZED ROS2 container
RUN apt update && apt install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    ros-humble-ament-cmake-auto \
    ros-humble-shape-msgs \
    ros-humble-ament-lint-auto

# Allow the new user to access the ZED SDK
RUN chown -R marsrover-zed:marsrover-zed /usr/local

WORKDIR /home/$NAME
USER $NAME

# Setup up tmuxp for launch scripts
USER root
RUN apt update && apt install -y tmux
USER $NAME
ENV PATH="$PATH:/home/$NAME/.local/bin"
RUN pip3 install --user tmuxp

# Set up automatic ROS2 sourcing and colorized output
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
RUN touch ~/.hushlogin

CMD ["/bin/bash"]