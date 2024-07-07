ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base AS base

# Development Tools
RUN DEBIAN_FRONTEND=noninteractive apt-get -y update \
    && apt-get -y install nano vim \
    && apt-get -y install iputils-ping iproute2 gettext \
    && rm -rf /var/lib/apt/lists/*

# System Dependencies
RUN DEBIAN_FRONTEND=noninteractive apt-get -y update \
    && sudo apt-get install -y libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

RUN DEBIAN_FRONTEND=noninteractive apt-get -y update \
    && apt-get install -y wget \
    && apt-get install -y unzip \
    && wget https://download.pytorch.org/libtorch/nightly/cpu/libtorch-shared-with-deps-latest.zip \
    && unzip libtorch-shared-with-deps-latest.zip \
    && rm -rf /var/lib/apt/lists/*

# Additional ROS packages
RUN DEBIAN_FRONTEND=noninteractive apt-get -y update \
    && apt-get -y install ros-${ROS_DISTRO}-teleop-twist-joy ros-${ROS_DISTRO}-robot-localization \
    && apt-get -y install ros-${ROS_DISTRO}-cv-bridge \
    && apt-get -y install ros-${ROS_DISTRO}-foxglove-bridge ros-${ROS_DISTRO}-rviz2 \
    && rm -rf /var/lib/apt/lists/*