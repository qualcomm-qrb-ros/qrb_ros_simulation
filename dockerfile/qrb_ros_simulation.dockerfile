FROM ros:jazzy-ros-base-noble

LABEL maintainer="Weijie Shen <weijshen@qti.qualcomm.com>"
LABEL description="this docker file is for running QRB ROS Simulation on host."

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# install and config ssh
RUN apt-get update \
    && apt-get install -y openssh-server \
    && sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

