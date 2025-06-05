
# `ros2in1` demo

This package contains a demo to prove usage of `ros2in1_support` package.
The tool aids the transition from ROS 1 to ROS 2.

## Overview

This repository is an example of how `turtlesim` from
[`ros_tutorials`](https://github.com/ros/ros_tutorials) is adapted to
expose some interfaces to ROS 2 world using `ros2in1_support`.

This document explains the demo and the steps required to do a seamless
transition from ROS 1 to ROS 2 based on the most well known tutorials code.

## Structure of this repository

The structure of this repository shows how a ROS 1 workspace can migrate
to ROS 2 by keeping the code base within the typical `<ws>/src` and adding
the elements required to it:

```
.
├── docker
├── docker-compose.yml
├── external             # External projects
│   ├── ros2in1_support  # ROS2in1 requirement
│   └── ros_tutorials    # Just an example of a ROS 1 app
├── README.md
└── src
    ├── ros1_package_a
    ├── ros1_package_b
    ├── ros2in1_support -> ../external/ros2in1_support    # simlink
    └── turtlesim -> ../external/ros_tutorials/turtlesim  # simlink
```

## Docker in this repo

The demo is run using docker compose. A docker image is produced from
a `Dockerfile` provided, which is based on `ros:noetic`, as an example of
the most current ROS 1 installation the reader may have.
On top of the base image, some requirements are installed, such as dependencies
of the `turtlesim`, `catkin tools` for convenience, and a fresh `ros-galactic`
installation, the latest in Ubuntu Focal on which `ros:noetic` is based.

### Requirement

Since this demo requires docker compose, it should be installed before starting.
[These instructions](https://docs.docker.com/compose/install/)
can serve as a guide to install it, by Docker documentation.

### Build docker image

Before running the container, the reader may build it personalizing it for
its own user.

```shell
docker compose build --build-arg UID=$UID
```

### Run a terminal

To create a terminal on with the environment prepared for the demo, this
command may provide the correct environment:

```shell
docker compose run --rm -it ros2in1_demo bash
```

## Q&A

Nothing reported yet.
