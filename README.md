
# `ros2in1` demo

This package contains a demo to prove usage of
[`ros2in1_support`](https://github.com/Intermodalics/ros2in1_support) package.
The tool aids the transition from ROS 1 to ROS 2.

**Note**:
Remember to fetch submodules by `git submodule update --init --recursive`.

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
    ├── ros2in1_support -> ../external/ros2in1_support    # symlink
    └── turtlesim -> ../external/ros_tutorials/turtlesim  # symlink
```

## Docker in this repo

The demo is run using docker compose. A docker image is produced from
a `Dockerfile` provided, which is based on `ros:noetic`, as an example of
the most current ROS 1 installation the reader may have.
On top of the base image, some requirements are installed, such as dependencies
of the `turtlesim`, `catkin tools` for convenience, and a fresh `ros-galactic`
installation, the latest in Ubuntu Focal on which `ros:noetic` is based.

The combination of Ubuntu Focal with ROS 1 noetic and ROS 2 galactic is the most
updated that can be used to work with an installation from sources. However,
this combination is already outdated compared to the current state of development
of ROS 2.

The user is encouraged to use a more recent version of ROS 2, e.g. Jazzy with
the current Ubuntu 24.04, even when that means to build ROS 1 from sources.

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

Then, topics and services can be tested in the second demo environment terminal, as explained next.

### Testing ROS topics

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

### Testing ROS services

The ROS 2 services offered by the demo may be listed by, e.g.:

```shell
ros2in1/ros2in1_demo$ ros2 service list
/sim/describe_parameters
/sim/get_parameter_types
/sim/get_parameters
/sim/list_parameters
/sim/set_parameters
/sim/set_parameters_atomically
/turtlesim1/clear
/turtlesim1/kill
/turtlesim1/reset
/turtlesim1/spawn
/turtlesim1/turtle1/set_pen
/turtlesim1/turtle1/teleport_absolute
/turtlesim1/turtle1/teleport_relative
/turtlesim2/clear
/turtlesim2/kill
/turtlesim2/reset
/turtlesim2/spawn
/turtlesim2/turtle1/set_pen
/turtlesim2/turtle1/teleport_absolute
/turtlesim2/turtle1/teleport_relative
```

A ROS 2 service may be called by, e.g.:

```shell
ros2in1/ros2in1_demo$ ros2 service call /turtlesim1/turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{ x: 4.0, y: 3.0, theta: 1.8}"
requester: making request: turtlesim.srv.TeleportAbsolute_Request(x=4.0, y=3.0, theta=1.8)

response:
turtlesim.srv.TeleportAbsolute_Response()
```

The turtle of the `/turtlesim1` namespace should have been translated to the new absolute pose.

The same list of services as viewed from ROS 1 is:

```shell
ros2in1/ros2in1_demo$ rosservice list
/rosout/get_loggers
/rosout/set_logger_level
/turtlesim1/clear
/turtlesim1/kill
/turtlesim1/reset
/turtlesim1/sim/get_loggers
/turtlesim1/sim/set_logger_level
/turtlesim1/spawn
/turtlesim1/turtle1/set_pen
/turtlesim1/turtle1/teleport_absolute
/turtlesim1/turtle1/teleport_relative
/turtlesim2/clear
/turtlesim2/kill
/turtlesim2/reset
/turtlesim2/sim/get_loggers
/turtlesim2/sim/set_logger_level
/turtlesim2/spawn
/turtlesim2/turtle1/set_pen
/turtlesim2/turtle1/teleport_absolute
/turtlesim2/turtle1/teleport_relative
```

## Code changes explained

The code changes made to `ros_tutorials` repository can be listed with the compare function of GitHub, as in:
[here](https://github.com/Intermodalics/ros_tutorials/compare/noetic-devel...ros2in1_tutorial/noetic-devel).

This section will explain the changes made.

### `package.xml`

The new build dependency `ros2in1_support` has been added to the package.

```xml
    <build_depend>ros2in1_support</build_depend>
```

This change tells ROS that the `turtlesim` package now depends on `ros2in1_support`. This change
is needed in all the packages that depends on `turtlesim`.

### `CMakeLists.txt`

The package that uses `ros2in1_support` needs to a) find that package, and 2) find all the ROS 2
dependencies that contain ROS 2 symbols that will be added to the ROS 1 application.
These symbols may be derived from interfaces, e.g. `geometry_msgs`, or by `rclcpp`, .e.g. `rclcpp::Node`.

These two needs are covered by calling `find_package(ros2in1 REQUIRED COMPONENTS <depA> <depB> ...)`:

```cmake
# Find ROS 2 packages
find_package(ros2in1_support REQUIRED COMPONENTS
    geometry_msgs
    std_msgs
    std_srvs
    rclcpp
    turtlesim
)
```

The libraries or executables provided by the package that requires ROS2in1 support need to build 1) against `ros2in1_support` itself, and 2) against targets found by `ros2in1_support` from the ROS 2 packages.

```cmake
include_directories(${ros2in1_support_INCLUDE_DIRS})

target_link_libraries(turtlesim_node Qt5::Widgets ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  ${ros2in1_support_TARGETS} ${ros2in1_support_LIBRARIES} # Newly added
)
```

To offer ROS 2 support, we can check for `ROS_SUPPORT` parameter. Furthermore, some conditional code will be added for the special case of using Galactic as the ROS 2 version chosen. That code has to do with the technique to remove signal handlers by ROS 2. It can be dealt with by:

```cmake
if(ROS2_SUPPORT)
  message(STATUS "[turtlesim] Using ROS 2 support")
  add_definitions(-DROS2_SUPPORT)
  if ("x$ENV{ROS2_DISTRO}" STREQUAL "xgalactic" OR
    "x${ROS2_DISTRO}" STREQUAL "xgalactic"
  )
    message(STATUS "[turtlesim] Using ROS2 distro special case: galactic")
    add_definitions(-DROS2_DISTRO_galactic)
  else()
  endif()
endif()
```

Finally, because `tutlesim` package provides interfaces, although it goes against
[ROS 2 best practices about interfaces](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html#background),
this package also provides conversions between ROS 1 and ROS 2 types.
Therefore, the newly added `include/turtlesim/conversions/turtlesim.h` file needs to be installed:

```cmake
install(
  DIRECTORY include/${PROJECT_NAME}/conversions
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
```

### Main entrypoint: `turtlesim.cpp`

This file has been modified to spawn a thread that runs a ROS 2 executor in the case that ROS 2 support is activated.
The conditional code allows to switch off ROS 2 support, and keep the application behavior identical to the original code.

In the headers section, some ROS 2 headers are conditionally loaded:

```cpp
#include <ros2in1_support/node.h>
#ifdef ROS2_SUPPORT
#  include <rclcpp/rclcpp.hpp>
#endif
```

ROS 2 is conditionally initialized next to ROS 1:

```cpp
    ros::init(...)
    // ...

#ifdef ROS2_SUPPORT
  {
    rclcpp::InitOptions ros2_init_options;
#if ROS2_DISTRO_galactic
    ros2_init_options.shutdown_on_sigint = false;
    rclcpp::init(argc, argv, ros2_init_options);
    rclcpp::uninstall_signal_handlers();
#else
    ros2_init_options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, ros2_init_options, rclcpp::SignalHandlerOptions::None);
#endif
  }
#endif
```

and finally, the executor is spawned in the `exec()` function of the `turtlesim` application, which is the function that originally calls also for the generation of the GUI and the call to ROS 1 event loop.

```cpp
  int exec()
  {
    // Start executor for ROS 2
#ifdef ROS2_SUPPORT
    std::unique_ptr<std::thread> ros2_executor_thread_;
    ros2_executor_thread_.reset(new std::thread([]() {
      rclcpp::Node::SharedPtr node = ros2in1_support::getRos2Node();
      rclcpp::spin(node);
    }));
#endif
    turtlesim::TurtleFrame frame;
    frame.show();
    return QApplication::exec();
  }
```

### Custom interfaces conversions

Turtlesim uses custom interfaces: 2 messages and 5 services:

* Messages
  * `Color`
  * `Pose`
* Services
  * `Kill`
  * `Spawn`
  * `SetPen`
  * `TeleportAbsolute`
  * `TeleportRelative`

It also uses a standard service `std_srvs/Empty`.

Since this same package provides the `turtlesim/*` services, it is convenient to provide the conversions between ROS 1 and ROS 2 required in the same package. This is done by the header file `include/turtlesim/conversions/turtlesim.h` file.

The whole file is made conditional to the use of ROS 2 support. In case that the package is built without support, the contents of the file are a No-Op.
The structure of the file is as follows:

1. Add headers of both ROS 1 and ROS 2 interfaces
2. Messages
  * a. Forward declaration of the structure `Ros2MessageType`
  * b. Specialization of the trait `Ros2MessageType::type` for every message
  * c. Definitions of template function `convert_2_to_1<>` specialization for every message
  * d. Definitions of template function `convert_1_to_2<>` specialization for every message
3. Services
  * a. Forward declaration of the structure `Ros2ServiceType`
  * b. Specialization of the trait `Ros2ServiceType::type` for every type
  * c. Definitions of template function `convert_2_to_1<>` specialization for every request message
  * d. Definitions of template function `convert_1_to_2<>` specialization for every response message

These are manually defined, and the body of the template functions can rely on other `convert_x_to_y<>()` functions to facilitate conversions and avoid code duplication.

### Code headers `turtle.h` and `turtle_frame.h`

There are two types of modifications in these headers:

* adding `ros2in1_support` headers and conversions headers
* change `ros::Publisher` and `ros::ServiceServer` by its templated counterparts from `ros2in1_support`.

Headers:

```cpp
# include <ros2in1_support/conversions/geometry_msgs.h>
# include <turtlesim/conversions/turtlesim.h>
# include <ros2in1_support/tf.h>
# include <ros2in1_support/publisher.h>
# include <ros2in1_support/service_server.h>
```

Notice that the conversions headers are added before the `ros2in1_support/publisher.h`, and `ros2in1_support/service_server.h`.

Publisher and service servers:

```cpp
  ros2in1_support::Publisher<Pose> pose_pub_;
  ros2in1_support::Publisher<Color> color_pub_;
  ros2in1_support::ServiceServer<turtlesim::SetPen> set_pen_srv_;
  ros2in1_support::ServiceServer<turtlesim::TeleportRelative> teleport_relative_srv_;
  ros2in1_support::ServiceServer<turtlesim::TeleportAbsolute> teleport_absolute_srv_;
```

These member variables are now templated, where in ROS 1 do not require the type of the message or service used. The types can be learned from the callback functions or `.publish()` arguments used.

### Implementation files `turtle.cpp` and `turtle_frame.cpp`

The change in these files are kept to minimal. `ros2in1_support` publishers and service servers need to be advertised with the member function `advertise()` where the first argument is the node handle, instead of been advertised by the ROS 1 `ros::NodeHandle::advertise()` or `ros::NodeHandle::advertiseService()` counterparts ([ref](https://docs.ros.org/en/noetic/api/roscpp/html/classros_1_1NodeHandle.html)).

```cpp
  pose_pub_.advertise(nh_, "pose", 1);
  color_pub_.advertise(nh_, "color_sensor", 1);
  set_pen_srv_.advertise(nh_, "set_pen", &Turtle::setPenCallback, this);
  teleport_relative_srv_.advertise(nh_, "teleport_relative", &Turtle::teleportRelativeCallback, this);
  teleport_absolute_srv_.advertise(nh_, "teleport_absolute", &Turtle::teleportAbsoluteCallback, this);
```

## Q&A

Nothing reported yet.
