# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear
import os
import yaml
import xacro
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    robot_gazebo_pkg = 'qrb_ros_sim_gazebo'
    gz_pkg_share = FindPackageShare(package=robot_gazebo_pkg).find(robot_gazebo_pkg)
    nodes = []
    launch_config_path = os.path.join(gz_pkg_share,'config','rml_63_gripper_launch_config.yaml')
    with open(launch_config_path, 'r') as f:
        config = yaml.safe_load(f)
        robot_group_args = config.get('robot_group_args', {})
        for robot_config in robot_group_args:
            nodes.extend(generate_robot_nodes(robot_config['arm_ns']))

    ld = LaunchDescription([
        *nodes,
    ])
    return ld

def generate_robot_nodes(arm_ns):
    load_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace=arm_ns,
        arguments=[
            'joint_state_broadcaster',
            ],
    )

    load_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace=arm_ns,
        arguments=[
            'rm_group_controller',
            ],
    )
    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace=arm_ns,
        arguments=[
            'hand_controller',
            ],
    )

    close_evt1 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
    )
    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_gripper_controller],
            )
    )

    return [
        load_joint_state_controller,
        close_evt1,
        close_evt2,
    ]
