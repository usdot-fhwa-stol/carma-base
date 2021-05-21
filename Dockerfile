#  Copyright (C) 2018-2021 LEIDOS.
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
FROM nvidia/cudagl:11.3.0-devel-ubuntu20.04

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
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}

# Specify which driver libraries/binaries will be mounted inside the container
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
    
# Avoid interactive prompts during the building of this docker image
ARG DEBIAN_FRONTEND="noninteractive"

RUN apt-get update && apt-get install -y lsb-release && apt-get clean ALL

# Install ROS Noetic
ARG ROS_DISTRO=noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \ 
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \ 
    && apt-get update \ 
    && apt-get install ros-noetic-desktop-full python3-rosinstall -y

# Prepare for ROS 2 Foxy installation
RUN apt update && apt install locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8
RUN apt update && apt install curl gnupg2 lsb-release \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Foxy
RUN apt update && apt install ros-foxy-desktop -y

RUN apt-get update && apt-get install -y \
        apt-transport-https \
        apt-utils \
        automake \
        autotools-dev \
        curl \
        dialog \
        gcovr \
        gdb \
        git \
        gnuplot-qt \
        less \
        libboost-python-dev \
        libfftw3-dev \
        libgeographic-dev \ 
        libgmock-dev \
        libpcap-dev \
        libpugixml-dev \
        mesa-utils \
        nano \
        nodejs \
        python3-catkin-pkg \
        python3-catkin-tools \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        python3-setuptools \
        python3-vcstool \
        ros-noetic-costmap-2d \
        ros-noetic-geodesy \
        ros-noetic-rosbridge-server \
        ros-noetic-rosserial \
        ros-noetic-rosserial-arduino \
        software-properties-common \
        sqlite3 \
        ssh \
        sudo \
        tmux \
        unzip \
        vim \
        x-window-system

RUN pip3 install -U testresources setuptools

# Install simple-pid
RUN pip3 install simple-pid

# Install AutonomouStuff dependencies
RUN sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list' && \
        apt-get update && \
        apt-get install -y libas-common

# Install KVaser CAN
RUN apt-add-repository -y ppa:astuff/kvaser-linux && \
    apt-get update -qq && \
    apt-get install -y kvaser-canlib-dev \
        can-utils kvaser-drivers-dkms

# Add carma user
ENV USERNAME carma
RUN useradd -m $USERNAME && \
        echo "$USERNAME:$USERNAME" | chpasswd && \
        usermod --shell /bin/bash $USERNAME && \
        usermod -aG sudo $USERNAME && \
        mkdir -p /etc/sudoers.d && \
        echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
        chmod 0440 /etc/sudoers.d/$USERNAME && \
        usermod  --uid 1000 $USERNAME && \
        groupmod --gid 1000 $USERNAME
RUN mkdir -p /opt/carma && chown carma:carma -R /opt/carma
USER carma

ADD --chown=carma package.xml /home/carma/.base-image/workspace/src/carma_base/
ADD --chown=carma entrypoint.sh /home/carma/.base-image/
ADD --chown=carma init-env.sh /home/carma/.base-image/

RUN sudo rosdep init && \
        rosdep update && \
        rosdep install --from-paths ~/.base-image/workspace/src --ignore-src -y

# Export QT X11 Forwarding variables
RUN sudo echo 'export QT_X11_NO_MITSHM=1' >> /home/carma/.base-image/init-env.sh

RUN sudo git clone --depth 1 https://github.com/vishnubob/wait-for-it.git ~/.base-image/wait-for-it &&\
    sudo mv ~/.base-image/wait-for-it/wait-for-it.sh /usr/bin 

# Install Armadillo
RUN cd ~/ && \
        curl -L  http://sourceforge.net/projects/arma/files/armadillo-9.800.1.tar.xz > armadillo-9.800.1.tar.xz && \
        tar -xvf armadillo-9.800.1.tar.xz && \
        cd armadillo-9.800.1 && \
        ./configure && \
        make && \
        sudo make install && \
        cd ../ && \
        rm -R armadillo-9.800.1 armadillo-9.800.1.tar.xz

# Install VimbaSDK for the Mako cameras
RUN cd ~/ && \
        curl -L  https://github.com/usdot-fhwa-stol/avt_vimba_camera/raw/develop/Vimba_v3.1_Linux.tgz > Vimba_v3.1_Linux.tgz && \
        sudo tar -xzf ./Vimba_v3.1_Linux.tgz -C /opt && \
        cd /opt/Vimba_3_1/VimbaGigETL && \
        sudo ./Install.sh && \
        sudo echo 'export GENICAM_GENTL32_PATH=$GENICAM_GENTL32_PATH:/opt/Vimba_3_1/VimbaGigETL/CTI/x86_32bit/' >> /home/carma/.base-image/init-env.sh && \
        sudo echo 'export GENICAM_GENTL64_PATH=$GENICAM_GENTL64_PATH:/opt/Vimba_3_1/VimbaGigETL/CTI/x86_64bit/' >> /home/carma/.base-image/init-env.sh && \
        rm ~/Vimba_v3.1_Linux.tgz

# Set environment variable for SonarQube Binaries. Two binaries are will go into this directory:
#   - The Build Wrapper which executes a code build to capture C++
#   - The Sonar Scanner which uploads the results to SonarCloud
ENV SONAR_DIR=/opt/sonarqube

# Pull scanner from internet
RUN sudo mkdir $SONAR_DIR && \
        sudo curl -o $SONAR_DIR/sonar-scanner.zip https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-4.4.0.2170-linux.zip && \
        sudo curl -o $SONAR_DIR/build-wrapper.zip https://sonarcloud.io/static/cpp/build-wrapper-linux-x86.zip && \
        # Install Dependancy of NodeJs 6+
        sudo curl -sL https://deb.nodesource.com/setup_10.x | sudo bash - && \
        # Install JQ Json Parser Tool
        sudo mkdir /opt/jq && \
        sudo curl -L "https://github.com/stedolan/jq/releases/download/jq-1.5/jq-linux64" -o /opt/jq/jq && \
        sudo chmod +x /opt/jq/jq

# Unzip scanner
RUN cd $SONAR_DIR && \ 
        sudo unzip $SONAR_DIR/sonar-scanner.zip -d . && \
        sudo unzip $SONAR_DIR/build-wrapper.zip -d . && \
        # Remove zip files 
        sudo rm $SONAR_DIR/sonar-scanner.zip && \
        sudo rm $SONAR_DIR/build-wrapper.zip && \
        # Rename files 
        sudo mv $(ls $SONAR_DIR | grep "sonar-scanner-") $SONAR_DIR/sonar-scanner/ && \
        sudo mv $(ls $SONAR_DIR | grep "build-wrapper-") $SONAR_DIR/build-wrapper/ && \
        # Add scanner, wrapper, and jq to PATH
        sudo echo 'export PATH=$PATH:/opt/jq/:$SONAR_DIR/sonar-scanner/bin/:$SONAR_DIR/build-wrapper/' >> /home/carma/.base-image/init-env.sh

# Install gcovr for code coverage tests and add code_coverage script folder to path
RUN sudo apt-get -y install gcovr && \
        sudo echo 'export PATH=$PATH:/home/carma/.ci-image/engineering_tools/code_coverage/' >> /home/carma/.base-image/init-env.sh

# Add engineering tools scripts to image
ADD --chown=carma ./code_coverage /home/carma/.ci-image/engineering_tools/code_coverage

# Download, build, and install PROJ, a package for coordinate transformations
RUN sudo git clone https://github.com/OSGeo/PROJ.git /home/carma/PROJ --branch 6.2.1 && \
        cd /home/carma/PROJ && \
        sudo ./autogen.sh && \
        sudo ./configure && \
        sudo make && \
        sudo make install
        
# Download a cmake module for PROJ
RUN cd /usr/share/cmake-3.16/Modules && sudo curl -O https://raw.githubusercontent.com/mloskot/cmake-modules/master/modules/FindPROJ4.cmake

# Add CUDA path
RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64' >> ~/.bashrc     \ 
    && echo 'export PATH=$PATH:/usr/local/cuda/bin' >> ~/.bashrc \
    && echo 'export CUDA_BIN_PATH=/usr/local/cuda' >> ~/.bashrc

# Install pip futures to support rosbridge
RUN pip install future

# Final system setup. This must go last before the ENTRYPOINT
RUN mkdir -p /opt/carma/routes /opt/carma/logs /opt/carma/launch &&\
    echo "source ~/.base-image/init-env.sh" >> ~/.bashrc &&\
    echo "cd /opt/carma" >> ~/.bashrc 

ENTRYPOINT [ "/home/carma/.base-image/entrypoint.sh" ]
