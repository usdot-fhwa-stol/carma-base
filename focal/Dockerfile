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

# The parent docker image has CUDA support since some modules use GPU-based acceleration
FROM nvidia/cudagl:11.2.0-devel-ubuntu20.04

# Dockerfile directory relative to root
ARG DOCKERFILE_DIR="focal"

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
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-base"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

# Specify which platform GPUs are available inside the container
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} \
        # Specify which driver libraries/binaries will be mounted inside the container
        NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Avoid interactive prompts during the building of this docker image
ARG DEBIAN_FRONTEND="noninteractive"

ARG BASE_DEPS="ca-certificates \
        curl \
        gnupg2 \
        locales \
        lsb-release \
        openssl \
        python3-rosinstall \
        ros-noetic-desktop-full \
        xterm \
        libmsgsl-dev"

ARG ROS_DEPS="apt-transport-https \
        apt-utils \
        automake \
        autotools-dev \
        dialog \
        gcovr \
        gdb \
        git \
        gnuplot-qt \
        jq \
        less \
        libboost-python-dev \
        libfftw3-dev \
        libgeographic-dev \
        libgmock-dev \
        libnl-genl-3-dev \
        libopenblas-dev \
        libpcap-dev \
        libpugixml-dev \
        lttng-tools \
        lttng-modules-dkms \
        liblttng-ust-dev \
        mesa-utils \
        nano \
        nodejs \
        python3-babeltrace \
        python3-catkin-pkg \
        python3-catkin-tools \
        python3-colcon-common-extensions \
        python3-future \
        python3-lttng \
        python3-pip \
        python3-rosdep \
        python3-setuptools \
        python3-testresources \
        python3-vcstool \
        python3-pybind11 \
        ros-noetic-costmap-2d \
        ros-noetic-dataspeed-can \
        ros-noetic-dbw-mkz \
        ros-noetic-geodesy \
        ros-noetic-grid-map \
        ros-noetic-lusb \
        ros-noetic-rosserial \
        ros-noetic-rosserial-arduino \
        ros-noetic-velodyne-pcl \
        software-properties-common \
        sqlite3 \
        ssh \
        sudo \
        tmux \
        unzip \
        vim \
        wait-for-it \
        x-window-system"

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

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

COPY --chown=carma ${DOCKERFILE_DIR}/package.xml /home/carma/.base-image/workspace/src/carma_base/
COPY --chown=carma ./entrypoint.sh ${DOCKERFILE_DIR}/init-env.sh /home/carma/.base-image/
COPY --chown=carma ./code_coverage /home/carma/.ci-image/engineering_tools/code_coverage

# Install ROS Noetic
ARG ROS_DISTRO=noetic
RUN sed -i 's|http://archive.ubuntu.com|http://us.archive.ubuntu.com|g' /etc/apt/sources.list && \
        # Add ROS 1 repo
        apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
        sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(grep -oP "UBUNTU_CODENAME\=\K.*" /etc/os-release) main" > /etc/apt/sources.list.d/ros-latest.list' && \
        apt update && \
        apt-get install --no-install-recommends --yes ${BASE_DEPS} && \
        # Install Sonar dependency nodejs
        curl -sL https://deb.nodesource.com/setup_16.x | bash - && \
        # Install ROS dependencies
        apt-get install --no-install-recommends --yes ${ROS_DEPS} && \
        # Install AutonomouStuff dependencies
        sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list' && \
        apt-get update && \
        apt-get install --no-install-recommends --yes libas-common && \
        # Vimba Deps
        add-apt-repository --update --yes ppa:rock-core/qt4 && \
        apt-get install --fix-missing --no-install-recommends --yes libqtcore4 libqt4-network libqt4-qt3support && \
        # Install KVaser CAN
        add-apt-repository --update --yes ppa:astuff/kvaser-linux && \
        apt-get install --no-install-recommends --yes kvaser-canlib-dev can-utils && \
        # Download a cmake module for PROJ, needed for lanelet2_extension, autoware_lanelet2_ros_interface, and maybe more
        curl --output /usr/share/cmake-3.16/Modules/FindPROJ4.cmake https://raw.githubusercontent.com/mloskot/cmake-modules/master/modules/FindPROJ4.cmake && \
        # Install version 45.2.0 for setuptools since that is the latest version available for ubuntu focal
        # Version match is needed to build some of the packages
        pip3 install --no-cache-dir setuptools==45.2.0 simple-pid && \
        # Vimba source
        curl -L -o Vimba_v5.0_Linux.tgz https://github.com/usdot-fhwa-stol/avt_vimba_camera/raw/fix/update_vimba_sdk/Vimba_v5.0_Linux.tgz && \
        tar -xzf ./Vimba_v5.0_Linux.tgz -C /opt && \
        ./opt/Vimba_5_0/VimbaGigETL/Install.sh && \
        rm Vimba_v5.0_Linux.tgz && \
        # Set environment variable for SonarQube Binaries. Two binaries will go in this directory:
        #   - The Build Wrapper which executes a code build to capture C++
        #   - The Sonar Scanner which uploads the results to SonarCloud
        SONAR_DIR=/opt/sonarqube && \
        # Pull scanner from internet
        mkdir $SONAR_DIR && \
        curl -o $SONAR_DIR/sonar-scanner.zip https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-5.0.1.3006-linux.zip && \
        curl -o $SONAR_DIR/build-wrapper.zip https://sonarcloud.io/static/cpp/build-wrapper-linux-x86.zip && \
        # Unzip scanner
        unzip $SONAR_DIR/sonar-scanner.zip -d "$SONAR_DIR"/ && \
        unzip $SONAR_DIR/build-wrapper.zip -d "$SONAR_DIR"/ && \
        # Remove zip files
        rm $SONAR_DIR/sonar-scanner.zip && \
        rm $SONAR_DIR/build-wrapper.zip && \
        # Rename files
        mv "$SONAR_DIR"/sonar-scanner-* "$SONAR_DIR"/sonar-scanner/ && \
        mv "$SONAR_DIR"/build-wrapper-* "$SONAR_DIR"/build-wrapper/ && \
        # FIXME: The following symlink will no longer be required once images
        # that depend on carma-base change from wait-for-it.sh to wait-for-it
        ln -s /usr/bin/wait-for-it /usr/bin/wait-for-it.sh && \
        rosdep --rosdistro noetic init && \
        sudo -u carma rosdep --rosdistro noetic update && \
        sudo -u carma rosdep --rosdistro noetic install --from-paths /home/carma/.base-image/workspace/src --ignore-src -y && \
        sudo -u carma echo "source ~/.base-image/init-env.sh" >> /home/carma/.bashrc && \
        sudo -u carma echo "cd /opt/carma" >> /home/carma/.bashrc && \
        # Install Java 17
        apt-get install -y openjdk-17-jdk && \
        apt-get clean && \
        rm -rf /var/lib/apt/lists/*

USER carma

ENTRYPOINT [ "/home/carma/.base-image/entrypoint.sh" ]
