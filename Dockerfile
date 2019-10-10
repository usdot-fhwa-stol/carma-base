#  Copyright (C) 2018-2019 LEIDOS.
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

FROM ros:kinetic-ros-base 

ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-base"
LABEL org.label-schema.description="Base operating system install for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/CARMAPlatform"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

RUN apt-get update && \
        apt-get install -y \
        git \
        ssh \
        ros-kinetic-rosjava \
        ros-kinetic-rosbridge-server \
        sudo \
        tmux \
        vim \
        nano \
        less \
        apt-transport-https \
        python-catkin-pkg \
        python-rosdep \
        python3-pip \
        python3-colcon-common-extensions \
        python3-setuptools \
        python3-vcstool \
        nodejs \
        unzip \
        gcovr

# Add dependencies for other CARMA package
RUN apt-get install -y \
        ros-kinetic-jsk-recognition-msgs \
        ros-kinetic-pacmod-msgs \
        ros-kinetic-can-msgs \
        ros-kinetic-automotive-navigation-msgs \
        ros-kinetic-autoware-msgs \
        ros-kinetic-automotive-platform-msgs \
        ros-kinetic-pacmod3 \
        ros-kinetic-dbw-mkz-msgs \
        libpcap-dev \
        ros-kinetic-gps-common \
        ros-kinetic-swri-math-util \
        ros-kinetic-swri-roscpp \
        ros-kinetic-swri-serial-util \
        ros-kinetic-swri-string-util \
        ros-kinetic-swri-nodelet \
        ros-kinetic-message-runtime \
        ros-kinetic-message-generation \
        ros-kinetic-diagnostic-msgs \
        ros-kinetic-diagnostic-updater \
        ros-kinetic-nav-msgs \
        ros-kinetic-nodelet \
        ros-kinetic-novatel-gps-msgs \
        ros-kinetic-sensor-msgs \
        ros-kinetic-tf

RUN pip3 install -U setuptools

RUN sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list' && \
        apt-get update && \
        apt-get install -y ros-kinetic-astuff-sensor-msgs \
        libas-common

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
RUN rosdep update && \
        rosdep install --from-paths ~/.base-image/workspace/src --ignore-src -y

RUN sudo git clone --depth 1 https://github.com/vishnubob/wait-for-it.git ~/.base-image/wait-for-it &&\
    sudo mv ~/.base-image/wait-for-it/wait-for-it.sh /usr/bin 

# Final system setup
RUN mkdir -p /opt/carma/app/bin /opt/carma/params /opt/carma/routes /opt/carma/urdf /opt/carma/logs /opt/carma/launch /opt/carma/app/mock_data /opt/carma/app/engineering_tools /opt/carma/drivers &&\
    echo "source ~/.base-image/init-env.sh" >> ~/.bashrc &&\
    echo "cd /opt/carma" >> ~/.bashrc 

# Set environment variable for SonarQube Binaries
# Two binaries are will go in this repo. 
# The Build Wrapper which executes a code build to capture C++
# The Sonar Scanner which evaluates both C++ and Java then uploads the results to SonarCloud
ENV SONAR_DIR=/opt/sonarqube

# Pull scanner from internet
RUN sudo mkdir $SONAR_DIR && \
        sudo curl -o $SONAR_DIR/sonar-scanner.zip https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-3.3.0.1492-linux.zip && \
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

ENTRYPOINT [ "/home/carma/.base-image/entrypoint.sh" ]
