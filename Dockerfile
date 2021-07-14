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
FROM dustynv/ros:noetic-ros-base-l4t-r32.4.4

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
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES}
    
# Avoid interactive prompts during the building of this docker image
ARG DEBIAN_FRONTEND="noninteractive"

RUN apt-get update && apt-get install -y lsb-release && apt-get clean ALL



RUN apt-get update && apt-get install -y \
        apt-transport-https \
        apt-utils \
        libnl-genl-3-dev \
        libopenblas-dev \
        automake \
        autotools-dev \
        curl \
        dialog \
        gcovr \
        gdb \
        git \
        gnuplot-qt \
        less \
        libpython-dev \
        libboost-dev \
        libboost-python-dev \
        libeigen3-dev \
        libfftw3-dev \
        libgeographic-dev \ 
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

RUN . /opt/ros/noetic/setup.sh && \
        sudo rosdep fix-permissions && \
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


# Download, build, and install PROJ, a package for coordinate transformations
RUN sudo git clone https://github.com/OSGeo/PROJ.git /home/carma/PROJ --branch 6.2.1 && \
        cd /home/carma/PROJ && \
        sudo ./autogen.sh && \
        sudo ./configure && \
        sudo make && \
        sudo make install
        
# Download a cmake module for PROJ
RUN cd /usr/share/cmake-3.10/Modules && sudo curl -O https://raw.githubusercontent.com/mloskot/cmake-modules/master/modules/FindPROJ4.cmake

# Add CUDA path
RUN echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64' >> ~/.bashrc     \ 
    && echo 'export PATH=$PATH:/usr/local/cuda/bin' >> ~/.bashrc \
    && echo 'export CUDA_BIN_PATH=/usr/local/cuda' >> ~/.bashrc

# Install pip futures to support rosbridge
RUN pip3 install future

# Final system setup. This must go last before the ENTRYPOINT
RUN mkdir -p /opt/carma/routes /opt/carma/logs /opt/carma/launch &&\
    echo "source ~/.base-image/init-env.sh" >> ~/.bashrc &&\
    echo "cd /opt/carma" >> ~/.bashrc 

ENTRYPOINT [ "/home/carma/.base-image/entrypoint.sh" ]
