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
FROM nvidia/cudagl:11.2.0-devel-ubuntu20.04

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
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} \
        # Specify which driver libraries/binaries will be mounted inside the container
        NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
    
# Avoid interactive prompts during the building of this docker image
ARG DEBIAN_FRONTEND="noninteractive"

ARG AUTOWAREAUTO_DEPS="coinor-libipopt-dev \
        coinor-libipopt1v5 \
        libarmadillo-dev \
        libcgal-dev \
        libmumps-5.2.1 \
        libmumps-dev \
        libmumps-seq-5.2.1 \
        libmumps-seq-dev \
        libproj-dev \
        libscalapack-mpi-dev \
        libscalapack-openmpi-dev \
        libscalapack-openmpi2.1 \
        libscotch-6.0 \
        ros-foxy-acado-vendor \
        ros-foxy-ament-cmake-google-benchmark \
        ros-foxy-apex-test-tools \
        ros-foxy-automotive-platform-msgs \
        ros-foxy-casadi-vendor \
        ros-foxy-diagnostic-updater \
        ros-foxy-gps-msgs \
        ros-foxy-joy-linux \
        ros-foxy-lgsvl-msgs \
        ros-foxy-osqp-vendor \
        ros-foxy-osrf-testing-tools-cpp \
        ros-foxy-point-cloud-msg-wrapper \
        ros-foxy-ros-testing \
        ros-foxy-rosapi \
        ros-foxy-rosapi-msgs \
        ros-foxy-rosbridge-library \
        ros-foxy-rosbridge-msgs \
        ros-foxy-rosbridge-server \
        ros-foxy-tvm-vendor \
        ros-foxy-udp-driver \
        ros-foxy-udp-msgs \
        ros-foxy-yaml-cpp-vendor"

ARG BASE_DEPS="ca-certificates \
        curl \
        gnupg2 \
        locales \
        lsb-release \
        openssl \
        python3-rosinstall \
        ros-noetic-desktop-full"

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
        mesa-utils \
        nano \
        python3-catkin-pkg \
        python3-catkin-tools \
        python3-colcon-common-extensions \
        python3-future \
        python3-pip \
        python3-rosdep \
        python3-setuptools \
        python3-testresources \
        python3-vcstool \
        ros-foxy-desktop \
        ros-foxy-rmw-cyclonedds-cpp \
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

# Install ROS Noetic
ARG ROS_DISTRO=noetic
RUN sed -i 's|http://archive.ubuntu.com|http://us.archive.ubuntu.com|g' /etc/apt/sources.list && \
        # Add ROS 1 repo
        apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
        sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(grep -oP "UBUNTU_CODENAME\=\K.*" /etc/os-release) main" > /etc/apt/sources.list.d/ros-latest.list' && \
        # Add ROS 2 repo
        apt-key adv --fetch-keys https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc && \
        sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(grep -oP "UBUNTU_CODENAME\=\K.*" /etc/os-release) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
        apt-get update && \
        apt-get install --yes ${BASE_DEPS} && \
        # Prepare for ROS 2 Foxy installation
        locale-gen en_US en_US.UTF-8 && \
        update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
        export LANG=en_US.UTF-8 && \
        # Install ROS 2 Foxy
        apt-get install --yes ${AUTOWAREAUTO_DEPS} ${ROS_DEPS} && \
        # Vimba Deps
        add-apt-repository --update --yes ppa:rock-core/qt4 && \
        apt-get install --fix-missing --yes libqtcore4 libqt4-network libqt4-qt3support && \
        # Install KVaser CAN
        add-apt-repository --update --yes ppa:astuff/kvaser-linux && \
        apt-get install --yes kvaser-canlib-dev can-utils && \
        # Download a cmake module for PROJ, needed for lanelet2_extension, autoware_lanelet2_ros_interface, and maybe more
        curl --output /usr/share/cmake-3.16/Modules/FindPROJ4.cmake https://raw.githubusercontent.com/mloskot/cmake-modules/master/modules/FindPROJ4.cmake && \
        # Install version 45.2.0 for setuptools since that is the latest version available for ubuntu focal
        # Version match is needed to build some of the packages
        pip3 install --no-cache-dir setuptools==45.2.0 simple-pid

###
# TODO: The following sequence of commands make a local update to ament_cmake to resolve an issue 
#       with the default xml parsing. Once the PR https://github.com/ament/ament_cmake/pull/287 is  
#       backported to ROS 2 Foxy, this can be removed.
###
RUN sudo rm /opt/ros/foxy/share/ament_cmake_core/cmake/core/package_xml_2_cmake.py && \
    sudo curl -o /opt/ros/foxy/share/ament_cmake_core/cmake/core/package_xml_2_cmake.py https://raw.githubusercontent.com/ament/ament_cmake/efcbe328d001c9ade93a06bd8035642e37dd6f2a/ament_cmake_core/cmake/core/package_xml_2_cmake.py

# Install AutonomouStuff dependencies
RUN sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list' && \
        apt-get update && \
        apt-get install -y libas-common

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
        groupmod --gid 1000 $USERNAME && \
        mkdir -p /opt/carma/{launch,logs,routes} && \
        chown carma:carma -R /opt/carma
USER carma

COPY --chown=carma package.xml /home/carma/.base-image/workspace/src/carma_base/
COPY --chown=carma entrypoint.sh init-env.sh /home/carma/.base-image/
COPY --chown=carma ./code_coverage /home/carma/.ci-image/engineering_tools/code_coverage

RUN sudo rosdep init && \
        rosdep update && \
        rosdep install --from-paths ~/.base-image/workspace/src --ignore-src -y

# Install VimbaSDK for the Mako cameras
# Vimba source
RUN cd ~/ && \
        curl -L  https://github.com/usdot-fhwa-stol/avt_vimba_camera/raw/fix/update_vimba_sdk/Vimba_v5.0_Linux.tgz > Vimba_v5.0_Linux.tgz && \
        sudo tar -xzf ./Vimba_v5.0_Linux.tgz -C /opt && \
        cd /opt/Vimba_5_0/VimbaGigETL && \
        sudo ./Install.sh && \
        rm ~/Vimba_v5.0_Linux.tgz

# Set environment variable for SonarQube Binaries. Two binaries will go in this directory:
#   - The Build Wrapper which executes a code build to capture C++
#   - The Sonar Scanner which uploads the results to SonarCloud
RUN SONAR_DIR=/opt/sonarqube && \
        # Pull scanner from internet
        sudo mkdir $SONAR_DIR && \
        sudo curl -o $SONAR_DIR/sonar-scanner.zip https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-4.4.0.2170-linux.zip && \
        sudo curl -o $SONAR_DIR/build-wrapper.zip https://sonarcloud.io/static/cpp/build-wrapper-linux-x86.zip && \
        # Install Dependancy of NodeJs 6+
        sudo curl -sL https://deb.nodesource.com/setup_16.x | sudo bash - && \
        sudo apt-get install -y nodejs && \
        # Unzip scanner
        sudo unzip $SONAR_DIR/sonar-scanner.zip -d "$SONAR_DIR"/ && \
        sudo unzip $SONAR_DIR/build-wrapper.zip -d "$SONAR_DIR"/ && \
        # Remove zip files 
        sudo rm $SONAR_DIR/sonar-scanner.zip && \
        sudo rm $SONAR_DIR/build-wrapper.zip && \
        # Rename files 
        sudo mv "$SONAR_DIR"/sonar-scanner-* "$SONAR_DIR"/sonar-scanner/ && \
        sudo mv "$SONAR_DIR"/build-wrapper-* "$SONAR_DIR"/build-wrapper/ && \
        # FIXME: The following symlink will no longer be required once images
        # that depend on carma-base change from wait-for-it.sh to wait-for-it
        sudo ln -s /usr/bin/wait-for-it /usr/bin/wait-for-it.sh

# Install non-ros1 dependant version of catkin
# This can be used without issue for ROS2 builds wheras the noetic version has compatability issues
# install catkin_pkg
RUN cd $HOME && \
        mkdir catkin_ros2_ws && \
        cd catkin_ros2_ws && \
        git clone https://github.com/ros-infrastructure/catkin_pkg.git && \
        cd catkin_pkg && \
        # Checkout a known working commit
        git checkout 60096f4b4a0975774651122b7e2d346545f8098a && \
        python3 setup.py install --prefix $HOME/catkin --single-version-externally-managed --record foo --install-layout deb && \
        cd ../ && \
        # install catkin
        git clone https://github.com/ros/catkin.git && \
        cd catkin && \
        # Checkout a known working commit
        git checkout 085e8950cafa3eb979edff1646b9e3fe55a7053a && \
        mkdir build && \
        cd build && \
        PYTHONPATH=$HOME/catkin/lib/python3/dist-packages/ cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/catkin -DPYTHON_EXECUTABLE=/usr/bin/python3 && \
        make -j install && \
        # Result is installation under ~/catkin so use with 
        # source ~/cakin/setup.bash
        # Final system setup. This must go last before the ENTRYPOINT
        echo "source ~/.base-image/init-env.sh" >> ~/.bashrc && \
        echo "cd /opt/carma" >> ~/.bashrc

ENTRYPOINT [ "/home/carma/.base-image/entrypoint.sh" ]
