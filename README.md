
# QRB ROS SIMULATION

## Overview
`qrb ros simulation` is a package designed to set up the Qualcomm robotic simulation environment. 
- This project provides simulation configurations for various robotic systems, including robotic arms, AMRs, sensors, and more.
- This project allows for extensive testing and validation of robotic development without the need for physical prototypes.

## Quick Start

> **Note：**
> This document 's build & run is the latest.
> If it conflict with the online document, please follow this.

We provide one way to use this package on host machine.

<details>
<summary>ubuntu24.04</summary>

#### Setup
1. Please follow this [steps](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) to install ros-jazzy-desktop and setup ROS env.
2. Install gazebo with ROS and other dependencies
```bash
sudo apt-get install -y ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers
source /opt/ros/jazzy/setup.bash
```
3. Download qrb_ros_simulation and meshes files
```bash
mkdir -p ~/qrb_ros_simulation_ws
cd ~/qrb_ros_simulation_ws
git clone https://github.com/qualcomm-qrb-ros/qrb_ros_simulation.git
cd qrb_ros_simulation
chmod +x meshes_download.sh
./meshes_download.sh
```


#### Build
```bash
cd ~/qrb_ros_simulation_ws
colcon build
```

#### Run

```bash
source install/local_setup.sh
```

Four pre-configured robotic models are ready for immediate launch. 

##### RML-63 robotic arm
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
##### QRB Robot Base AMR
```bash
ros2 launch qrb_ros_sim_gazebo gazebo_robot_base.launch.py
```
##### QRB Robot Base AMR Mini
```bash
ros2 launch qrb_ros_sim_gazebo gazebo_robot_base_mini.launch.py
```
##### QRB Mobile Manipulator Robot
1. launch
```bash
ros2 launch qrb_ros_sim_gazebo gazebo_mobile_manipulator.launch.py
```
2. Press the `Play` button to start the simulation
3. In a separate terminal, load the controllers of Mobile Manipulator Robot
```bash
cd ~/qrb_ros_simulation_ws
source install/local_setup.sh
ros2 launch qrb_ros_sim_gazebo gazebo_mobile_manipulator_load_controller.launch.py
```
</details>
 

<br>

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

