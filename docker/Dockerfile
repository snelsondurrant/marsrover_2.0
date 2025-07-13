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

# Define a username and password
ARG NAME=marsrover-docker
ARG PASS=thekillpack

# Create the user
RUN useradd -ms /bin/bash "$NAME" \
    && usermod -aG sudo,video,dialout "$NAME" \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && groupadd docker \
    && usermod -aG docker "$NAME" \
    && echo "$NAME:$PASS" | chpasswd

#################################
# IMPORTANT DOCUMENTATION NOTE! #
#################################

# When adding dependencies or Dockerfile commands below, please document your changes
# following this format:
#
#     # What do these commands do? - Full Name, Month Year
#     # Why do we need them? Which ROS2 nodes require them?
#     < Add system dependencies here >
#
# https://docs.docker.com/reference/dockerfile/

# Install system dependencies
RUN apt-get update && apt-get upgrade -y --no-install-recommends \
    && apt-get install -y --no-install-recommends \
    # Install basic system utilities
    curl \
    wget \
    git \
    vim \
    nano \
    tmux \
    python3-pip \
    python3-venv \
    # Install graphical utilities
    x11-apps \
    x11-utils \
    x11-xserver-utils \
    xauth \
    # Install tools for terrain maps - Nelson Durrant, Apr 2025
    # Used for loading in and parsing GeoTIFF elevation maps
    libgdal-dev \
    python3-rasterio \
    && rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt-get update && apt-get upgrade -y --no-install-recommends \
    && apt-get install -y --no-install-recommends \
    # Install the navigation2 and robot_localization packages - Nelson Durrant, Feb 2025
    # Pre-built full navigation stack for GPS navigation and localization
    ros-humble-navigation2 \
    ros-humble-robot-localization \
    # Install graphical interface packages - Nelson Durrant, Feb 2025
    # We use RViz2, RQT, and Python3-tk for GUIs
    ros-humble-rviz2 \
    ros-humble-rqt* \
    python3-tk \
    # Install plotjuggler - Nelson Durrant, Feb 2025
    # Useful for plotting and visualizing data from ROS2 bags and real-time tests
    ros-humble-plotjuggler-ros \
    # Install the aruco_opencv packages - Nelson Durrant, Feb 2025
    # Used for detecting ArUco markers in the simulation and real world
    ros-humble-aruco-opencv \
    # Install mobility packages - Nelson Durrant, Feb 2025
    # Used for controlling the robot's movement through a joystick or keyboard
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-teleop-twist-keyboard \
    ros-humble-topic-tools \
    # Install turtlesim tf2 packages - Nelson Durrant, Feb 2025
    # Used in the ROS2 and tf2 tutorials
    ros-humble-turtle-tf2-py \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-turtlesim \
    # Install pcl_conversions - Nelson Durrant, Mar 2025
    # Used for point cloud conversion in the unitree lidar package
    ros-humble-pcl-conversions \
    # Install xacro convertor package - Nelson Durrant, Mar 2025
    # Used for converting the rover's xacro files to urdf files
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-urdf-launch \
    # Install zed_msgs package - Nelson Durrant, Mar 2025
    # Official ROS2 interfaces for the ZED camera
    ros-humble-zed-msgs \
    # Install MoveIt2 package - Nelson Durrant, Jun 2025
    # Open-source robotic arm manipulation platform (for use on the rover?)
    ros-humble-moveit \
    && rm -rf /var/lib/apt/lists/*

# Install architecture-specific system dependencies
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    apt-get update && apt-get install -y --no-install-recommends \
    # Install gazebo and turtlebot3 packages - Nelson Durrant, Feb 2025
    # We use Gazebo Classic for the simulator and TurtleBot3 for the robot model
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3-gazebo \
    # Install mapviz packages - Nelson Durrant, Feb 2025
    # Used for visualizing the robot's path and gps location
    ros-humble-mapviz \
    ros-humble-mapviz-plugins \
    ros-humble-tile-map \
    && rm -rf /var/lib/apt/lists/*; \
fi

USER $NAME
WORKDIR /home/$NAME

# Install python dependencies
RUN pip3 install --no-cache-dir \
    # Install python UTM library - Nelson Durrant, Feb 2025
    # Used for converting object and aruco poses to GPS coordinates
    utm \
    # Install ascii art packages - Nelson Durrant, Mar 2025
    # Used for the Mars Rover ASCII art display on startup
    --upgrade pillow \
    asciimatics \
    # Install tmux docker-compose-esque launcher - Nelson Durrant, Mar 2025
    # Used for launching multiple tmux windows with a single command
    tmuxp \
    # Install pyserial - Nelson Durrant, Mar 2025
    # Used for serial communication with the Arduinos
    pyserial \
    # Install mapproxy tile caching deps - Nelson Durrant, Mar 2025
    # Used for pre-caching map tiles for rover test locations
    mercantile \
    requests \
    # Install AStar planning package - Nelson Durrant, Apr 2025
    # Simple A* path planning algorithm implementation for 2D maps
    astar \
    # Install fastmcp and uv - Nelson Durrant, July 2025
    # Fast MCP server and launch system for hooking the rover controls into an LLM (experimental)
    fastmcp \
    uv

# Install PlatformIO
RUN curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py \
    && python3 get-platformio.py \
    && rm get-platformio.py

# Set up PlatformIO shell command symlinks
USER root
RUN mkdir -p /home/$NAME/.platformio/penv/bin \
    && ln -s /home/$NAME/.platformio/penv/bin/platformio /usr/local/bin/platformio \
    && ln -s /home/$NAME/.platformio/penv/bin/pio /usr/local/bin/pio \
    && ln -s /home/$NAME/.platformio/penv/bin/piodebuggdb /usr/local/bin/piodebuggdb
USER $NAME

# Set up automatic ROS2 sourcing, colorized output, and add local python binaries to PATH
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc \
    && touch ~/.hushlogin
ENV PATH="$PATH:/home/$NAME/.local/bin"
