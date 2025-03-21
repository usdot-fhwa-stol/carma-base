# CARMA Base Image Docker Configuration Script

FROM nvidia/cudagl:11.2.0-devel-ubuntu20.04

ARG DOCKERFILE_DIR="noetic"

ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ARG DEBIAN_FRONTEND="noninteractive"

# Step 1: Set up the carma user
RUN useradd -m carma && \
    echo "carma:carma" | chpasswd && \
    usermod --shell /bin/bash carma && \
    usermod -aG sudo carma && \
    mkdir -p /etc/sudoers.d && \
    echo "carma ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/carma && \
    chmod 0440 /etc/sudoers.d/carma && \
    usermod  --uid 1000 carma && \
    groupmod --gid 1000 carma && \
    mkdir -p /opt/carma/launch /opt/carma/logs /opt/carma/routes && \
    chown carma:carma -R /opt/carma

# Copy dependency files
COPY ${DOCKERFILE_DIR}/base_deps.txt ${DOCKERFILE_DIR}/additional_deps.txt ${DOCKERFILE_DIR}/ros_deps.txt /tmp/

# Step 2: Install base dependencies
RUN apt-get update && \
    xargs -a /tmp/base_deps.txt apt-get install --no-install-recommends -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Step 3: Install additional dependencies
RUN apt-get update && \
    xargs -a /tmp/additional_deps.txt apt-get install --no-install-recommends -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Step 4: Set up ROS repository and install ROS dependencies
ARG ROS_DISTRO=noetic
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-get update && \
    xargs -a /tmp/ros_deps.txt apt-get install --no-install-recommends -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Step 5: Install AutonomouStuff dependencies
RUN echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list && \
    apt-get update && \
    apt-get install --no-install-recommends -y libas-common && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Step 6: Install Vimba dependencies
RUN add-apt-repository --update -y ppa:rock-core/qt4 && \
    apt-get update && \
    apt-get install --fix-missing --no-install-recommends -y \
    libqtcore4 \
    libqt4-network \
    libqt4-qt3support && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Step 7: Install KVaser CAN
RUN add-apt-repository --update -y ppa:astuff/kvaser-linux && \
    apt-get update && \
    apt-get install --no-install-recommends -y \
    kvaser-canlib-dev \
    can-utils && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Step 8: Download cmake module for PROJ and install Python dependencies
RUN curl --output /usr/share/cmake-3.16/Modules/FindPROJ4.cmake https://raw.githubusercontent.com/mloskot/cmake-modules/master/modules/FindPROJ4.cmake && \
    pip3 install --no-cache-dir \
    setuptools==45.2.0 \
    simple-pid

# Step 9: Install Vimba source
RUN curl -L -o Vimba_v5.0_Linux.tgz https://github.com/usdot-fhwa-stol/avt_vimba_camera/raw/fix/update_vimba_sdk/Vimba_v5.0_Linux.tgz && \
    tar -xzf ./Vimba_v5.0_Linux.tgz -C /opt && \
    ./opt/Vimba_5_0/VimbaGigETL/Install.sh && \
    rm Vimba_v5.0_Linux.tgz

# Step 10: Set up SonarQube
RUN SONAR_DIR=/opt/sonarqube && \
    mkdir $SONAR_DIR && \
    curl -o $SONAR_DIR/sonar-scanner.zip https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-5.0.1.3006-linux.zip && \
    curl -o $SONAR_DIR/build-wrapper.zip https://sonarcloud.io/static/cpp/build-wrapper-linux-x86.zip && \
    unzip $SONAR_DIR/sonar-scanner.zip -d "$SONAR_DIR"/ && \
    unzip $SONAR_DIR/build-wrapper.zip -d "$SONAR_DIR"/ && \
    rm $SONAR_DIR/sonar-scanner.zip && \
    rm $SONAR_DIR/build-wrapper.zip && \
    mv "$SONAR_DIR"/sonar-scanner-* "$SONAR_DIR"/sonar-scanner/ && \
    mv "$SONAR_DIR"/build-wrapper-* "$SONAR_DIR"/build-wrapper/

# Step 11: Install Java 17
RUN apt-get update && \
    apt-get install -y openjdk-17-jdk && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Step 12: Update to latest Git
RUN add-apt-repository ppa:git-core/ppa -y && \
    apt update && apt install git -y

# Step 13: Copy files general files for image
COPY --chown=carma ${DOCKERFILE_DIR}/package.xml /home/carma/.base-image/workspace/src/carma_base/
COPY --chown=carma ./entrypoint.sh ${DOCKERFILE_DIR}/init-env.sh /home/carma/.base-image/
COPY --chown=carma ./code_coverage /home/carma/.ci-image/engineering_tools/code_coverage

# Step 14: Set up ROS environment
RUN rosdep --rosdistro noetic init && \
    sudo -u carma rosdep --rosdistro noetic update && \
    sudo -u carma rosdep --rosdistro noetic install --from-paths /home/carma/.base-image/workspace/src --ignore-src -y && \
    sudo -u carma echo "source ~/.base-image/init-env.sh" >> /home/carma/.bashrc && \
    sudo -u carma echo "cd /opt/carma" >> /home/carma/.bashrc

# Clean up dependency files
RUN rm /tmp/base_deps.txt /tmp/additional_deps.txt /tmp/ros_deps.txt

USER carma

# Metadata arguments
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

ENTRYPOINT [ "/home/carma/.base-image/entrypoint.sh" ]
