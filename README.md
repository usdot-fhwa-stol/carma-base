| Dockerhub Noetic Build - Develop | Dockerhub Noetic Build - Release | Dockerhub Humble Build - Develop | Dockerhub Humble Build - Release |
|-------------------------------|------------------------------|------------------------------|------------------------------|
[![Noetic Build](https://github.com/usdot-fhwa-stol/carma-base/actions/workflows/dockerhub-noetic.yml/badge.svg?branch=develop)](https://github.com/usdot-fhwa-stol/carma-base/actions/workflows/dockerhub-noetic.yml) | [![Noetic Build](https://github.com/usdot-fhwa-stol/carma-base/actions/workflows/dockerhub-noetic.yml/badge.svg?tag=carma-system-*)](https://github.com/usdot-fhwa-stol/carma-base/actions/workflows/dockerhub-noetic.yml) | [![Humble Build](https://github.com/usdot-fhwa-stol/carma-base/actions/workflows/dockerhub-humble.yml/badge.svg?branch=develop)](https://github.com/usdot-fhwa-stol/carma-base/actions/workflows/dockerhub-humble.yml) | [![Humble Build](https://github.com/usdot-fhwa-stol/carma-base/actions/workflows/dockerhub-humble.yml/badge.svg?tag=carma-system-*)](https://github.com/usdot-fhwa-stol/carma-base/actions/workflows/dockerhub-humble.yml)

# CARMABase
CARMA Base is the Docker base image for Dockerized releases and deployments of the CARMA Platform. All CARMA images should inherit from this and any dependencies of those images should be instaled in this base image to minimize system build time and final image size.

Currently there are two CARMA Base image types. One is based on Ubuntu Focal Fossa (20.04 LTS) which supports ROS Noetic. 
The other is based on Ubuntu Jammy Jellyfish (22.04 LTS) which supports ROS 2 Humble.

# CARMAPlatform
The primary CARMAPlatform repository can be found [here](https://github.com/usdot-fhwa-stol/carma-platform) and is part of the [USDOT FHWA STOL](https://github.com/usdot-fhwa-stol/)
github organization. Documentation on how the CARMAPlatform functions, how it will evolve over time, and how you can contribute can be found at the above links as well

## Contribution 
Welcome to the CARMA contributing guide. Please read this guide to learn about our development process, how to propose pull requests and improvements, and how to build and test your changes to this project. [CARMA Contributing Guide](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/Contributing.md) 

## Code of Conduct 
Please read our [CARMA Code of Conduct](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/Code_of_Conduct.md) which outlines our expectations for participants within the CARMA community, as well as steps to reporting unacceptable behavior. We are committed to providing a welcoming and inspiring community for all and expect our code of conduct to be honored. Anyone who violates this code of conduct may be banned from the community.

## Attribution
The development team would like to acknowledge the people who have made direct contributions to the design and code in this repository. [CARMA Attribution](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/ATTRIBUTION.txt) 

## License
By contributing to the Federal Highway Administration (FHWA) Connected Automated Research Mobility Applications (CARMA), you agree that your contributions will be licensed under its Apache License 2.0 license. [CARMA License](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/docs/License.md) 

## Contact
Please click on the CARMA logo below to visit the Federal Highway Adminstration(FHWA) CARMA website.

[![CARMA Image](https://raw.githubusercontent.com/usdot-fhwa-stol/carma-platform/develop/docs/image/CARMA_icon.png)](https://highways.dot.gov/research/research-programs/operations/CARMA)
