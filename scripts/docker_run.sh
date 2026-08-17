#!/bin/bash

# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

docker_name="qrb_ros_simulation"
docker_tag="latest"

# override the default VNC password by exporting VNC_PASSWORD before running
# this script, e.g. `VNC_PASSWORD=mypassword ./scripts/docker_run.sh`
vnc_password="${VNC_PASSWORD:-qrbrossim}"

echo "Open http://<host_ip>:6080/vnc.html in your browser to access the desktop (password: ${vnc_password})"

docker run -it --rm \
  --name ${docker_name}_container \
  --network host \
  -e VNC_PASSWORD="${vnc_password}" \
	${docker_name}:${docker_tag}
