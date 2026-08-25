# Pull the ROS base image from the AWS ECR Public mirror of Docker official images
ARG ROS_BASE_IMAGE=public.ecr.aws/docker/library/ros:jazzy-ros-base
FROM ${ROS_BASE_IMAGE}

LABEL maintainer="Weijie Shen <weijshen@qti.qualcomm.com>"
LABEL description="this docker file is for running QRB ROS Simulation on host."

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# install remote desktop (XFCE4) + web-based VNC client (noVNC) so Gazebo/RViz
# GUIs can be used from a browser
RUN apt-get update && apt-get install -y --no-install-recommends \
    xfce4 \
    xfce4-terminal \
    dbus-x11 \
    iproute2 \
    tigervnc-standalone-server \
    tigervnc-common \
    tigervnc-tools \
    novnc \
    websockify \
    && rm -rf /var/lib/apt/lists/*

# no GPU is passed into the container, force Mesa's software rasterizer
# (llvmpipe) so OpenGL apps (Gazebo/RViz) can still initialize a context
ENV LIBGL_ALWAYS_SOFTWARE=1

ENV RESOLUTION=1920x1080

RUN mkdir -p /root/.vnc \
    && printf '#!/bin/bash\nunset SESSION_MANAGER\nunset DBUS_SESSION_BUS_ADDRESS\nexec dbus-launch --exit-with-session startxfce4\n' > /root/.vnc/xstartup \
    && chmod +x /root/.vnc/xstartup

COPY dockerfile/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

EXPOSE 5901 6080

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
