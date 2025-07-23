
# QRB ROS SIMULATION

## Overview
`qrb ros simulation` is a package designed to set up the Qualcomm robotic simulation environment. 
- This project provides simulation configurations for various robotic systems, including robotic arms, AMRs, sensors, and more.
- This project allows for extensive testing and validation of robotic development without the need for physical prototypes.

## Quick Start

> **Note：**
> This document 's build & run is the latest.
> If it conflict with the online document, please follow this.

You can use this package on host.

#### Download qrb_ros_simulation and meshes files

```bash
mkdir -p ~/qrb_ros_simulation_ws
cd ~/qrb_ros_simulation_ws
git clone https://github.com/qualcomm-qrb-ros/qrb_ros_simulation.git
cd qrb_ros_simulation
chmod +x scripts/meshes_download.sh
./scripts/meshes_download.sh
```

#### Setup

You can setup ROS2 Jazzy on your host machine with ubuntu24.04 OR you can use a Docker-based development environment directly.

<details>
<summary>ubuntu24.04</summary>

1. Please follow this [steps](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) to install ros-jazzy-desktop and setup ROS env.
2. Install gazebo with ROS and other dependencies
```bash
sudo apt-get install -y ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers
```

Next, you can follow the steps of [​**​Build**](#build)​​ and ​[​**Run**](#run) to launch the simulation.environment

</details>

<details>
<summary>docker</summary>

1. Build the docker image locally
```bash
cd qrb_ros_simulation
chmod +x scripts/docker_build.sh
./scripts/docker_build.sh
```
2. Start a docker container
```bash
chmod +x scripts/docker_run.sh
./scripts/docker_run.sh
```
3. In a separate terminal, copy the qrb_ros_simulation project to the docker container
```bash
docker cp ~/qrb_ros_simulation_ws qrb_ros_simulation_container:/root/qrb_ros_simulation_ws
```
4. Enable SSH service in the docker container
```bash
# set the password of user root
(docker) passwd
# enable SSH service
(docker) service ssh start
```
5. Login to the docker container by SSH
```bash
ssh -X -p 222 root@your_host_ip
```

Next, you can follow the steps of [​**​Build**](#build)​​ and ​[​**Run**](#run) to launch the simulation environment within the Docker container.

</details>

#### Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/qrb_ros_simulation_ws
colcon build
source install/local_setup.sh
```

#### Run

Four pre-configured robotic models are ready for immediate launch. 

<details>
<summary>RML-63 robotic arm</summary>

1. launch RML-63 robotic arm in gazebo
```bash
ros2 launch qrb_ros_sim_gazebo gazebo_rml_63_gripper.launch.py
```
2. Press the `Play` button to start the simulation
3. In a separate terminal, load the controllers of RML-63
```bash
cd ~/qrb_ros_simulation_ws
source install/local_setup.sh
ros2 launch qrb_ros_sim_gazebo gazebo_rml_63_gripper_load_controller.launch.py
```
</details>

<details>
<summary>QRB Robot Base AMR</summary>

```bash
ros2 launch qrb_ros_sim_gazebo gazebo_robot_base.launch.py
```
</details>

<details>
<summary>QRB Robot Base AMR Mini</summary>

```bash
ros2 launch qrb_ros_sim_gazebo gazebo_robot_base_mini.launch.py
```
</details>

<details>
<summary>QRB Mobile Manipulator Robot</summary>

1. launch
```bash
ros2 launch qrb_ros_sim_gazebo gazebo_mobile_manipulator.launch.py
```
2. Press the `Play` button to start the simulation
3. In a separate terminal, load the controllers of Mobile Manipulator Robot
```bash
cd ~/qrb_ros_simulation_ws
source install/local_setup.sh
ros2 launch qrb_ros_sim_gazebo gazebo_rml_63_gripper_load_controller.launch.py
```
</details>
</details>

You can get more details from [here](https://quic-qrb-ros.github.io/main/index.html).

## Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)


## Authors

* **Weijie Shen** - *Initial work* - [weijshen](https://github.com/quic-weijshen)

See also the list of [contributors](https://github.com/qualcomm-qrb-ros/qrb_ros_simulation/graphs/contributors) who participated in this project.


## License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.

