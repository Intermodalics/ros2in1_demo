
# `ros2in1` demo

This package contains a demo to prove usage of
[`ros2in1_support`](https://github.com/Intermodalics/ros2in1_support) package.
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

## Building

With a terminal within the demo environment, after following the steps of the
previous section, the code can be built with:

```shell
source /opt/ros/noetic/setup.bash
ROS2_DISTRO=galactic catkin build --cmake-args -DROS2_SUPPORT=ON -DRMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Testing

Once built the ROS 1 workspace, the example `turtlesim` is ready to publish both
in ROS 1 and ROS 2.

Let's open two terminals within the demo environment, something like:

```shell
# Substitute <uuid> by the UUID assigned by Docker to the demo container
docker exec -it demo-ros2in1_demo-run-<uuid> bash
```

In the first demo environment terminal, issue:

```shell
source /opt/ros/galactic/setup.bash
source ./devel/setup.bash
roslaunch turtlesim multisim.launch
```

In the second demo environment terminal, issue:

```shell
source /opt/ros/galactic/setup.bash
ros2 topic list
```

The ROS 2 topics published by the demo should be listed, e.g.:

```log
ros2in1/ros2in1_demo$ ros2 topic list
/parameter_events
/rosout
/turtlesim1/turtle1/color_sensor
/turtlesim1/turtle1/pose
/turtlesim2/turtle1/color_sensor
/turtlesim2/turtle1/pose
```

These topics can be subscribed by any newer ROS 2 node.
The topics are also available in ROS 1. Let's use a new terminal without ROS 2
environment contamination.

```shell
source ./devel/setup.bash
rostopic list
```

This would be a correct output:

```log
ros2in1/ros2in1_demo$ rostopic list
/rosout
/rosout_agg
/turtlesim1/turtle1/cmd_vel
/turtlesim1/turtle1/color_sensor
/turtlesim1/turtle1/pose
/turtlesim2/turtle1/cmd_vel
/turtlesim2/turtle1/color_sensor
/turtlesim2/turtle1/pose
```

These topics can be subscribed as usual by the rest of the ROS 1 application
pending migration.

## Q&A

Nothing reported yet.
