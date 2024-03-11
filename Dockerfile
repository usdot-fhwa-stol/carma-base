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

# The parent docker image with ROS Humble built for the Jetson Xavier
FROM dustynv/ros:humble-ros-base-l4t-r35.1.0

# Define arguments which are used in the following metadata definition
ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"

# Specify docker image metadata
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-base"
LABEL org.label-schema.description="Base operating system install for the CARMA Platform on ARM architecture"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-platform"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

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

COPY --chown=carma package.xml /home/carma/.base-image/workspace/src/carma_base/
COPY --chown=carma entrypoint.sh init-env.sh /home/carma/.base-image/
COPY --chown=carma ./code_coverage /home/carma/.ci-image/engineering_tools/code_coverage

# Layer for building ROS2 Humble packages from source
# NOTICE: needs to be chmod +x on host if not already
COPY install_pkgs.sh install_pkgs.sh
RUN ./install_pkgs.sh


USER carma

ENTRYPOINT [ "/home/carma/.base-image/entrypoint.sh" ]