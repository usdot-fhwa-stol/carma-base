#!/usr/bin/env bash

# This script builds ROS2 Humble packages from source that are required
# as dependencies for C1T components on ARM hardware

apt update

mkdir -p /opt/ros/humble/src
cd /opt/ros/src

rosinstall_generator --deps --rosdistro humble ros-base \
    acado_vendor \
    ament_cmake_google_benchmark \
    apex_test_tools \
    automotive_platform_msgs \
    casadi_vendor \
    diagnostic_updater \
    gps_msgs \ 
    joy_linux \
    lgsvl_msgs \
    osqp_vendor \
    osrf_testing_tools_cpp \
    point_cloud_msg_wrapper \
    ros_testing \
    rosapi \
    rosapi_msgs \
    rosbridge_library \
    rosbridge_msgs \
    rosbridge_server \
    tvm_vendor \
    udp_driver \
    udp_msgs \
    velodyne_pointcloud \
    yaml_cpp_vendor \
    rmw_cyclonedds_cpp \
    pcl_ros \
    test_msgs \
    nmea_msgs \
    gps_tools \
    rosbag2 \
> ros2.humble.ros-base.rosinstall
cat ros2.humble.ros-base.rosinstall
vcs import src < ros2.humble.ros-base.rosinstall

rosdep init
rosdep update
rosdep install - y \
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