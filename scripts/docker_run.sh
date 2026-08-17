#!/bin/bash

# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

docker_name="qrb_ros_simulation"
docker_tag="latest"
container_name="${docker_name}_container"

# override the default VNC password by exporting VNC_PASSWORD before running
# this script, e.g. `VNC_PASSWORD=mypassword ./scripts/docker_run.sh`
vnc_password="${VNC_PASSWORD:-qrbrossim}"

# the container is not removed when it exits, so the workspace built inside it
# and the downloaded Gazebo Fuel models are kept. reuse it when it already
# exists instead of failing on the duplicated container name.
if [ -n "$(docker ps -aq -f name="^${container_name}$")" ]; then
  echo "Reusing the existing container ${container_name}"
  echo "(the VNC password is the one it was created with, VNC_PASSWORD has no effect here)"
  docker start ${container_name}
else
  docker run -d \
    --name ${container_name} \
    --network host \
    -e VNC_PASSWORD="${vnc_password}" \
    ${docker_name}:${docker_tag}
fi

echo "Open http://<host_ip>:6080/vnc.html in your browser to access the desktop"
