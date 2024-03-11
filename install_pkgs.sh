#!/usr/bin/env bash

# This script builds ROS2 Humble packages from source that are required
# as dependencies for C1T components on ARM hardware

apt update

export ROS_PACKAGE_PATH='/opt/ros/humble/install/share'
export PYTHONPATH='/opt/ros/humble/install/lib/python3.8/site-packages/:$PYTHONPATH'
export CMAKE_PREFIX_PATH='/opt/ros/humble/install'

mkdir -p /opt/ros/humble/src
cd /opt/ros/humble

rosinstall_generator --deps --rosdistro humble \
    acado_vendor \
    casadi_vendor \
    diagnostic_updater \
    joy_linux \
    osrf_testing_tools_cpp \
    point_cloud_msg_wrapper \
    ros_testing \
    rosapi \
    rosapi_msgs \
    rosbridge_library \
    rosbridge_msgs \
    rosbridge_server \
    udp_driver \
    udp_msgs \
    velodyne_pointcloud \
    yaml_cpp_vendor \
    pcl_ros \
    test_msgs \
    nmea_msgs \
    rosbag2 \
    --exclude RPP \
> ros2.humble.ros_base.rosinstall
vcs import src < ros2.humble.ros_base.rosinstall

rosdep update
rosdep install -y \
    --ignore-src \
    --from-paths src \
    --rosdistro humble

colcon build \
    --merge-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

rm -rf /opt/ros/humble/src
rm -rf /opt/ros/humble/logs
rm -rf /opt/ros/humble/build
rm /opt/ros/humble/*.rosinstall

rm -rf /var/lib/apt/lists/*
apt-get clean