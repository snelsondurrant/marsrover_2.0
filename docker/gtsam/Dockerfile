# Created by Nelson Durrant, July 2025
FROM byuawesomerover/marsrover_2.0

ARG DEBIAN_FRONTEND=noninteractive
ARG NAME=marsrover-docker

# Install GTSAM (localization testing) - Nelson Durrant, July 2025
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake libboost-dev \
    && rm -rf /var/lib/apt/lists/*
USER $NAME

WORKDIR /home/$NAME
RUN git clone --depth 1 --branch 4.2 https://github.com/borglab/gtsam.git \
    && cd gtsam \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DGTSAM_WITH_TBB=ON \
    && make -j$(nproc) \
    && sudo make install \
    && sudo ldconfig \
    && cd /home/$NAME \
    && rm -rf gtsam