#!/bin/bash

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


# Sets all environment variables and sources scripts necessary for CARMA to operate

export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib/

[ -f "/opt/ros/kinetic/setup.bash" ] && source /opt/ros/kinetic/setup.bash
[ -f "/opt/autoware.ai/ros/install/setup.bash" ] && source /opt/autoware.ai/ros/install/setup.bash
[ -f "/opt/carma/install/setup.bash" ] && source /opt/carma/install/setup.bash
[ -f "/opt/carma/vehicle/config/carma.env" ] && source /opt/carma/vehicle/config/carma.env # Always source environment variables as last step
