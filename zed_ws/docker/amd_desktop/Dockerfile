# Created by Braden Meyers, May 2025
ARG UBUNTU_MAJOR=22
ARG UBUNTU_MINOR="04"
ARG CUDA_MAJOR=12
ARG CUDA_MINOR=6
ARG CUDA_PATCH=3
ARG ZED_SDK_MAJOR=4
ARG ZED_SDK_MINOR=2
ARG ZED_SDK_PATCH=3

ARG IMAGE_NAME=nvcr.io/nvidia/cuda:${CUDA_MAJOR}.${CUDA_MINOR}.${CUDA_PATCH}-devel-ubuntu${UBUNTU_MAJOR}."${UBUNTU_MINOR}"

FROM ${IMAGE_NAME}

ARG UBUNTU_MAJOR=22
ARG UBUNTU_MINOR=04
ARG CUDA_MAJOR=12
ARG CUDA_MINOR=6
ARG CUDA_PATCH=3
ARG ZED_SDK_MAJOR=4
ARG ZED_SDK_MINOR=2
ARG ZED_SDK_PATCH=3

ARG ROS2_DIST=humble       # ROS2 distribution

ARG DEBIAN_FRONTEND=noninteractive

ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}compute,video,utility
    
# Disable apt-get warnings
RUN apt-get update || true && apt-get install -y --no-install-recommends apt-utils dialog curl && \
  rm -rf /var/lib/apt/lists/*

# Create the ZED SDK link
ARG CUDA_MINOR_DW=1
RUN if [ "$CUDA_MAJOR" = "12" ] && [ "$CUDA_MINOR" != "1" ]; then \
      CUDA_MINOR_DW=1; \
    fi
RUN if [ "$CUDA_MAJOR" = "11" ] && [ "$CUDA_MINOR" != "8" ]; then \
      CUDA_MINOR_DW=8; \
    fi
ENV ZED_SDK_URL="https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/ZED_SDK_Ubuntu${UBUNTU_MAJOR}_cuda${CUDA_MAJOR}.${CUDA_MINOR_DW}_v${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}.${ZED_SDK_PATCH}.zstd.run"

# Check that this SDK exists
RUN echo "SDK link: $ZED_SDK_URL"
RUN if [ "$(curl -I "${ZED_SDK_URL}" -o /dev/null -s -w '%{http_code}\n' | head -n 1)" = "200" ]; then \
        echo "The URL points to something."; \
    else \
        echo "The URL does not point to a .run file or the file does not exist."; \
        exit 1; \
    fi


ENV TZ=Europe/Paris

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \ 
  apt-get update && \
  apt-get install --yes lsb-release wget less udev sudo build-essential cmake python3 python3-dev python3-pip python3-wheel git jq libopencv-dev libpq-dev zstd usbutils && \    
  rm -rf /var/lib/apt/lists/*

############ Install ROS2 ############

# Set and Check Locale
RUN apt-get update || true && \
  apt-get install --no-install-recommends -y locales && \
  locale-gen en_US en_US.UTF-8 && \
  update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
  export LANG=en_US.UTF-8 && \
  locale  # verify settings && \
  rm -rf /var/lib/apt/lists/*

# Setup Sources
RUN apt-get update || true && \
  apt-get install --no-install-recommends -y software-properties-common && \
  add-apt-repository universe && \
  apt-get install -y curl && \
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
  rm -rf /var/lib/apt/lists/*

# Install ROS2 Base packages and Python dependencies
RUN apt update &&\
  apt install --no-install-recommends -y \
  ros-${ROS2_DIST}-ros-base \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools && \
  pip3 install \
  argcomplete \
  numpy \
  empy \
  lark && \
  rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install the ZED SDK
RUN echo "CUDA Version $CUDA_VERSION" > /usr/local/cuda/version.txt

# Setup the ZED SDK
RUN apt-get update -y || true && \
  apt-get install --no-install-recommends dialog bash-completion lsb-release wget less udev sudo  build-essential cmake zstd python3 python3-pip libpng-dev libgomp1 -y && \
  python3 -m pip install --upgrade pip; python3 -m pip install numpy opencv-python-headless && \
  wget -q -O ZED_SDK_Linux_Ubuntu.run ${ZED_SDK_URL} && \
  chmod +x ZED_SDK_Linux_Ubuntu.run && \
  ./ZED_SDK_Linux_Ubuntu.run -- silent skip_cuda && \
  ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so && \
  rm ZED_SDK_Linux_Ubuntu.run && \
  rm -rf /var/lib/apt/lists/*

# Install the ZED ROS2 Wrapper
ENV ROS_DISTRO ${ROS2_DIST}

# # Copy the sources in the Docker image
# WORKDIR /root/ros2_ws/src
# COPY tmp_sources/ ./

# RUN ls -lah /root/ros2_ws/src/
# WORKDIR /root/ros2_ws/

# RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
#   apt-get update -y || true && rosdep update && \
#   rosdep install --from-paths src --ignore-src -r -y && \
#   colcon build --parallel-workers $(nproc) --symlink-install \
#   --event-handlers console_direct+ --base-paths src \
#   --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
#   ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
#   ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' " && \
#   rm -rf /var/lib/apt/lists/*

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

# Install missing dependencies from the basic ZED ROS2 container
RUN apt update && apt install -y \
    ros-humble-zed-msgs \
    ros-humble-xacro \
    ros-humble-point-cloud-transport \
    ros-humble-cob-srvs \
    ros-humble-geographic-msgs \
    ros-humble-nmea-msgs \
    ros-humble-robot-localization \
    ros-humble-image-transport \     
    ros-humble-diagnostic-updater \
    && rm -rf /var/lib/apt/lists/*

# Define a username and password for the new user
ARG NAME=marsrover-zed
ARG PASS=marsrover

# Set up a new user
RUN useradd -ms /bin/bash $NAME
RUN usermod -aG sudo $NAME
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN usermod -aG video $NAME
RUN usermod -aG dialout $NAME
RUN groupadd docker
RUN usermod -aG docker $NAME
RUN echo "$NAME:$PASS" | chpasswd

# Allow the new user to access the ZED SDK
RUN chown -R $NAME:$NAME /usr/local/

# Setup up tmuxp for launch scripts
USER root
RUN apt update && apt install -y tmux
USER $NAME
ENV PATH="$PATH:/home/$NAME/.local/bin"
RUN pip3 install --user tmuxp

# Setup environment variables
# COPY ros_entrypoint.sh /sbin/ros_entrypoint.sh
# RUN sudo chmod 755 /sbin/ros_entrypoint.sh

# Set up automatic ROS2 sourcing and colorized output
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
RUN touch ~/.hushlogin

CMD ["/bin/bash"]