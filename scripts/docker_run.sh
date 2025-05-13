#!/bin/bash

# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

docker_name="qrb_ros_simulation"
docker_tag="latest"

docker run -it --rm \
  --name ${docker_name}_container \
  -p 222:22 \
	${docker_name}:${docker_tag}
