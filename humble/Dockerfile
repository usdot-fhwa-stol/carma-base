# CARMA Base Image Docker Configuration Script

FROM nvidia/cuda:11.7.1-devel-ubuntu22.04

# Avoid interactive prompts during the building of this docker image
ARG DEBIAN_FRONTEND="noninteractive"

# Dockerfile directory relative to root
ARG DOCKERFILE_DIR="humble"

# Specify which platform GPUs are available inside the container
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Copy package lists early
COPY ${DOCKERFILE_DIR}/ros_packages.txt ${DOCKERFILE_DIR}/additional_packages.txt /tmp/

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    ca-certificates curl gnupg2 lsb-release

# Set up ROS2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install AutonomouStuff dependencies
RUN sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'

# Install ROS2 packages
RUN apt-get update && \
    apt-get install -y $(cat /tmp/ros_packages.txt)

# Install additional packages
RUN apt-get update && \
    apt-get install -y $(cat /tmp/additional_packages.txt)

# Update to latest Git
RUN add-apt-repository ppa:git-core/ppa -y && \
    apt update && apt install git -y

# Set up PROJ
RUN curl -o /usr/share/cmake-3.22/Modules/FindPROJ.cmake https://raw.githubusercontent.com/OSGeo/libgeotiff/refs/heads/master/libgeotiff/cmake/FindPROJ.cmake

# Install Python packages
RUN apt-get update && apt-get install -y python3-pip
RUN pip3 install --no-cache-dir vcstool setuptools pybind11

# Install Java 17
RUN apt-get update && apt-get install -y openjdk-17-jdk

# Clean
RUN rm -rf /var/lib/apt/lists/*

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

# Set up ROS 2 environment
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
    mkdir -p /opt/carma/launch /opt/carma/logs /opt/carma/routes && \
    chown carma:carma -R /opt/carma

# Copy files for carma user
COPY --chown=carma ./entrypoint.sh ${DOCKERFILE_DIR}/init-env.sh /home/carma/.base-image/
COPY --chown=carma ./code_coverage /home/carma/.ci-image/engineering_tools/code_coverage

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

USER carma
WORKDIR /home/carma

# Set the entrypoint
ENTRYPOINT [ "/home/carma/.base-image/entrypoint.sh" ]
