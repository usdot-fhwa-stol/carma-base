# CARMABase Noetic (ROS Noetic)
CARMA Base is the Docker base image for Dockerized releases and deployments of the CARMA Platform. All CARMA images should inherit from this and any dependencies of those images should be instaled in this base image to minimize system build time and final image size.

Currently this CARMA Base image is built on Ubuntu 20.04 LTS (Focal Fossa). It supports ROS Noetic.