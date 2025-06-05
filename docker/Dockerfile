
FROM ros:noetic

ARG USER_NAME
ARG UID=1000

RUN rm /etc/apt/sources.list.d/ros1-latest.list \
    && apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
      curl \
      wget \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-latest-archive-keyring.gpg \
    && echo "deb [ signed-by=/usr/share/keyrings/ros-latest-archive-keyring.gpg ] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
      > /etc/apt/sources.list.d/ros1.list \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-latest-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2.list \
    && apt update \
    && DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends \
      python3-catkin-tools \
      git \
      ros-noetic-tf2 \
      ros-noetic-tf2-geometry-msgs \
      ros-galactic-ros-base \
    && git clone -b noetic-devel https://github.com/ros/ros_tutorials.git /tmp/ros_tutorials \
    && /opt/ros/noetic/env.sh rosdep update \
    && /opt/ros/noetic/env.sh rosdep install -y --ignore-src --from-paths /tmp/ros_tutorials \
    && rm -rf /tmp/ros_tutorials \
    && rm -rf ${HOME}/.ros \
    && apt clean \
    && rm -rf /var/cache/apt/archives /var/lib/apt/lists/*

RUN adduser --disabled-password --gecos '' --shell /bin/bash --uid ${UID} ${USER_NAME} \
    && usermod -G adm,dialout,sudo,dip,src,video,plugdev ${USER_NAME} \
    && echo "${USER_NAME}:ubuntu" | chpasswd \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/${USER_NAME}_conf

USER ${USER_NAME}
