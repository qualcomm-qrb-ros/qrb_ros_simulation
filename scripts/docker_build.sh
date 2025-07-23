#!/bin/bash

# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

current_dir=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
docker_name="qrb_ros_simulation"
docker_tag="latest"

docker build \
  -t ${docker_name}:${docker_tag} \
  -f ${current_dir}/../dockerfile/qrb_ros_simulation.dockerfile .
