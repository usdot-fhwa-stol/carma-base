#!/bin/bash

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

# Sets all environment variables and sources scripts necessary for CARMA to operate

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib/

# Export QT X11 Forwarding variables
export QT_X11_NO_MITSHM=1

# Set Cyclone DDS as default RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Add scanner and wrapper to PATH
export PATH=$PATH:/opt/sonarqube/sonar-scanner/bin/:/opt/sonarqube/build-wrapper/

# Add code_coverage script folder to path for gcovr
export PATH=$PATH:/home/carma/.ci-image/engineering_tools/code_coverage/

# Add CUDA paths
export CUDA_HOME=/usr/local/cuda
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CUDA_HOME/lib64
export PATH=$PATH:$CUDA_HOME/bin

# Source ROS2 Humble setup
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source CARMA setup if available
if [ -f "/opt/carma/install/setup.bash" ]; then
    source /opt/carma/install/setup.bash
fi

# Always source environment variables as last step
if [ -f "/opt/carma/vehicle/config/carma.env" ]; then
    source /opt/carma/vehicle/config/carma.env
fi