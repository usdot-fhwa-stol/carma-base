#  Copyright (C) 2018-2024 LEIDOS.
#
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

# CARMA Base Image Docker Configuration Script

# Use NVIDIA CUDA image as the base
FROM nvidia/cuda:11.7.1-devel-ubuntu22.04

# Define arguments which are used in the following metadata definition
ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"

# Specify docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-base"
LABEL org.label-schema.description="Base operating system install for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-platform"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

# Specify which platform GPUs are available inside the container
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Avoid interactive prompts during the building of this docker image
ARG DEBIAN_FRONTEND="noninteractive"

# Update the AUTOWAREAUTO_DEPS for Humble (if available)
ARG AUTOWAREAUTO_DEPS="ros-humble-acado-vendor \
        ros-humble-ament-cmake-google-benchmark \
        ros-humble-apex-test-tools \
        ros-humble-automotive-platform-msgs \
        ros-humble-casadi-vendor \
        ros-humble-diagnostic-updater \
        ros-humble-gps-msgs \
        ros-humble-joy-linux \
        ros-humble-lgsvl-msgs \
        ros-humble-osqp-vendor \
        ros-humble-osrf-testing-tools-cpp \
        ros-humble-point-cloud-msg-wrapper \
        ros-humble-ros-testing \
        ros-humble-rosapi \
        ros-humble-rosapi-msgs \
        ros-humble-rosbridge-library \
        ros-humble-rosbridge-msgs \
        ros-humble-rosbridge-server \
        ros-humble-tvm-vendor \
        ros-humble-udp-driver \
        ros-humble-udp-msgs \
        ros-humble-velodyne-pointcloud \
        ros-humble-yaml-cpp-vendor"

# Update ROS_DEPS for Humble
ARG ROS_DEPS="python3-colcon-common-extensions \
        ros-humble-desktop \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-pcl-ros \
        ros-humble-test-msgs \
        ros-humble-nmea-msgs \
        ros-humble-gps-tools \
        ros-humble-rosbag2-storage-mcap"

# Additional non-ROS1 specific dependencies
ARG ADDITIONAL_DEPS="ca-certificates \
        curl \
        gnupg2 \
        locales \
        lsb-release \
        openssl \
        git \
        jq \
        libboost-python-dev \
        libfftw3-dev \
        libgeographic-dev \
        libgmock-dev \
        libnl-genl-3-dev \
        libopenblas-dev \
        libpcap-dev \
        libpugixml-dev \
        lttng-tools \
        mesa-utils \
        nano \
        nodejs \
        python3-pip \
        python3-setuptools \
        python3-vcstool \
        python3-pybind11 \
        software-properties-common \
        sqlite3 \
        ssh \
        sudo \
        tmux \
        unzip \
        vim \
        wait-for-it \
        x-window-system"

# Install dependencies
RUN apt-get update && apt-get install -y \
        ${ADDITIONAL_DEPS} \
        && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
        && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
        && apt-get update \
        && apt-get install -y ${AUTOWAREAUTO_DEPS} ${ROS_DEPS} \
        && apt-get clean \
        && rm -rf /var/lib/apt/lists/*

# Install Java 17
RUN apt-get update && apt-get install -y openjdk-17-jdk \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install SonarQube scanner
RUN SONAR_DIR=/opt/sonarqube \
    && mkdir $SONAR_DIR \
    && curl -o $SONAR_DIR/sonar-scanner.zip https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-5.0.1.3006-linux.zip \
    && curl -o $SONAR_DIR/build-wrapper.zip https://sonarcloud.io/static/cpp/build-wrapper-linux-x86.zip \
    && unzip $SONAR_DIR/sonar-scanner.zip -d "$SONAR_DIR"/ \
    && unzip $SONAR_DIR/build-wrapper.zip -d "$SONAR_DIR"/ \
    && rm $SONAR_DIR/sonar-scanner.zip $SONAR_DIR/build-wrapper.zip \
    && mv "$SONAR_DIR"/sonar-scanner-* "$SONAR_DIR"/sonar-scanner/ \
    && mv "$SONAR_DIR"/build-wrapper-* "$SONAR_DIR"/build-wrapper/

# Set up ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Add carma user
RUN useradd -m carma && \
    echo "carma:carma" | chpasswd && \
    usermod --shell /bin/bash carma && \
    usermod -aG sudo carma && \
    mkdir -p /etc/sudoers.d && \
    echo "carma ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/carma && \
    chmod 0440 /etc/sudoers.d/carma && \
    usermod  --uid 1000 carma && \
    groupmod --gid 1000 carma && \
    mkdir -p /opt/carma/{launch,logs,routes} && \
    chown carma:carma -R /opt/carma

USER carma
WORKDIR /home/carma

# Set the entrypoint
ENTRYPOINT ["/bin/bash"]