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

FROM nvidia/cudagl:10.0-devel-ubuntu16.04

ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-base"
LABEL org.label-schema.description="Base operating system install for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-platform"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
    
RUN apt-get update && apt-get install -y lsb-release && apt-get clean ALL        

ENV ROS_DISTRO kinetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'     \ 
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654     \ 
    && apt-get update     \ 
    && apt-get install  \
       openssl \
       ca-certificates \
       ros-kinetic-desktop-full \
       python-rosinstall -y

RUN apt-get update && \
        DEBIAN_FRONTEND=noninteractive apt-get install -y \
        sudo \
        curl \
        git \
        ssh \
        ros-kinetic-rosbridge-server \
        tmux \
        vim \
        nano \
        less \
        apt-transport-https \
        bc \
        python-catkin-pkg \
        python-rosdep \
        python-pip \
        python3-pip \
        python3-colcon-common-extensions \
        python3-setuptools \
        python3-vcstool \
        nodejs \
        unzip \
        gcovr \
        libpcap-dev \
        libfftw3-dev \
        gnuplot-qt \
        libgeographic-dev \ 
        libpugixml-dev \
        python-catkin-tools \
        libboost-python-dev \
        sqlite3 \
        autotools-dev \
        automake \
        ros-kinetic-rosserial-arduino \
        ros-kinetic-rosserial \
        dialog \
        x-window-system \
        mesa-utils \
        gdb \
        software-properties-common

RUN pip3 install setuptools==51.3.3

# Install simple-pid
RUN pip install simple-pid

# Pull AutonomouStuff deps
RUN sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list' && \
        apt-get update && \
        apt-get install -y ros-kinetic-astuff-sensor-msgs \
        libas-common

# Install KVaser CAN
RUN apt-add-repository -y ppa:astuff/kvaser-linux && \
    apt-get update -qq && \
    apt-get install -y kvaser-canlib-dev \
    kvaser-drivers-dkms \
    ros-kinetic-kvaser-interface

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
        rosdep update --include-eol-distros && \
        rosdep install --from-paths ~/.base-image/workspace/src --ignore-src -y

# Export QT X11 Forwarding variables
RUN sudo echo 'export QT_X11_NO_MITSHM=1' >> /home/carma/.base-image/init-env.sh

RUN sudo git clone --depth 1 https://github.com/vishnubob/wait-for-it.git ~/.base-image/wait-for-it &&\
    sudo mv ~/.base-image/wait-for-it/wait-for-it.sh /usr/bin 

# Install Armadillo
RUN cd ~/ && \
        curl -Lk  http://sourceforge.net/projects/arma/files/armadillo-9.800.1.tar.xz > armadillo-9.800.1.tar.xz && \
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
        sudo echo 'export GENICAM_GENTL64_PATH=$GENICAM_GENTL64_PATH:/opt/Vimba_3_1/VimbaGigETL/CTI/x86_64bit/' >> /home/carma/.base-image/init-env.sh

# Set environment variable for SonarQube Binaries
# Two binaries will go in this repo:
# The Build Wrapper which executes a code build to capture C++
# The Sonar Scanner which uploads the results to SonarCloud
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

RUN sudo git clone https://github.com/OSGeo/PROJ.git /home/carma/PROJ --branch 6.2.1 && \
        cd /home/carma/PROJ && \
        sudo ./autogen.sh && \
        sudo ./configure && \
        sudo make && \
        sudo make install
        
RUN cd /usr/share/cmake-3.5/Modules && sudo curl -O https://raw.githubusercontent.com/mloskot/cmake-modules/master/modules/FindPROJ4.cmake

# Add cuda path
RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64' >> ~/.bashrc     \ 
    && echo 'export PATH=$PATH:/usr/local/cuda/bin' >> ~/.bashrc \
    && echo 'export CUDA_BIN_PATH=/usr/local/cuda' >> ~/.bashrc

# Install pip futures to support rosbridge
RUN pip install future

# Final system setup this must go last before the ENTRYPOINT
RUN mkdir -p /opt/carma/routes /opt/carma/logs /opt/carma/launch &&\
    echo "source ~/.base-image/init-env.sh" >> ~/.bashrc &&\
    echo "cd /opt/carma" >> ~/.bashrc 

ENTRYPOINT [ "/home/carma/.base-image/entrypoint.sh" ]
