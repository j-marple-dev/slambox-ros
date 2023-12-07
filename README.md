# SLAM BOX ros driver
SLAM BOX ros driver is a ROS package to communicate with SLAM BOX.

Table of Contents
=================

* [1. Getting started](#1-getting-started)
   * [1.1. Installation](#11-installation)
      * [1.1.1. Requirements](#111-requirements)
      * [1.1.2. (Optional) Building the docker image](#112-optional-building-the-docker-image)
      * [1.1.3. Local ROS system](#113-local-ros-system)
         * [Pre-requisite](#pre-requisite)
   * [1.2. Configuration](#12-configuration)
      * [1.2.1. Client configuration](#121-client-configuration)
   * [1.3. Usage](#13-usage)
* [2. Getting help](#2-getting-help)
* [3. Contributing](#3-contributing)
* [4. External resources](#4-external-resources)

# 1. Getting started
## 1.1. Installation
### 1.1.1. Requirements
- [ROS noetic](https://wiki.ros.org/noetic/Installation/Ubuntu) (Recommended)
- [SLAMBOX-SDK](https://github.com/j-marple-dev/SLAMBOX-SDK) (v0.1.0)
- glog (>=v0.6.0)
- CMake (>= 3.16.3)
- docker (Optional but highly recommended)
- `dialout` group permission. Use below command to include `dialout` group to your linux account for UART communication.
    ```shell
    sudo usermod -aG dialout $USER
    ```

### 1.1.2. (Optional) Building the docker image
* Our docker image includes development environment. We highly recommend docker system.
    ```shell
    # Clone this repository
    git clone https://github.com/j-marple-dev/slambox-ros.git --recursive

    # Build docker image
    docker build . -t jmarpledev/slambox_ros -f docker/Dockerfile  --build-arg UID=$(id -u) --build-arg GID=$(id -u)

    # Run docker container with shell (For development environment)
    docker run -tid --privileged -e DISPLAY=:0 -e TERM=xterm-256color -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v /dev:/dev -v $PWD:/home/user/catkin_ws/src/slambox-ros --network host jmarpledev/slambox_ros /usr/bin/zsh

    # Run docker container for running SLAMBOX driver client
    docker run -ti --privileged -e TERM=xterm-256color -v /dev:/dev -v $PWD:/home/user/catkin_ws/src/slambox-ros --network host jmarpledev/slambox_ros /usr/bin/bash -lic "roslaunch slambox_ros slambox_ros_client.launch"
    ```

### 1.1.3. Local ROS system

#### Pre-requisite

- **SLAMBOX-SDK**: Please follow installation instruction on https://github.com/j-marple-dev/SLAMBOX-SDK 


```shell
# Assuming that your ROS workspace is ~/catkin_ws
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/j-marple-dev/slambox-ros.git --recursive
cd ../
catkin build slambox_ros
# Choose the shell which you are using
source devel/setup.{bash|zsh}
```

## 1.2. Configuration
### 1.2.1. Client configuration
- Please refer to [config/client.yaml](config/client.yaml) for configuration on client side.
```yaml
serial_communication:
  enabled: true
  port_name: "/dev/ttyUSB0"
  baudrate: 921600

ethernet_communication:
  enabled: true
  server_addr: "192.168.1.50"
  port: 21580

publish:
  odom_topic: "/SLAMBOX/odom"
  pointcloud_topic: "/SLAMBOX/pointcloud"

subscribe:
  request_topic: "/SLAMBOX/request"
```

## 1.3. Usage
- Running ROS node
    ```shell
    roslaunch slambox_ros slambox_ros_client.launch
    ```

- Check rostopic in another shell
    ```shell
    rostopic hz /SLAMBOX/odom /SLAMBOX/pointcloud
    ```

# 2. Getting help
Please visit https://sbox.jmarple.ai for more information.

# 3. Contributing
- For those who wish to contribute to this proejct please refer to the [CONTRIBUING.md](CONTRIBUTING.md).

# 4. External resources
- [glog](https://github.com/google/glog) - Google Logging Library
- [fmt](https://github.com/fmtlib/fmt) Modern formatting library
- [cli](https://github.com/daniele77/cli) - C++ CLI Library
- [ducker](https://github.com/JeiKeiLim/ducker) - Docker Helper CLI application
