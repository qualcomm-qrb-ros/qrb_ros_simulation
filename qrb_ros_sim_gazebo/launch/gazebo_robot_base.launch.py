# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear
import os
import sys
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import gazebo_qrb_ros_sim_launch_utils as launch_utils

def generate_launch_description():
    robot_base_set = True
    camera_set = False
    robot_entity_name='qrb_amr_robot'
    launch_args = launch_utils.declare_common_launch_arguments(robot_base_set, camera_set, robot_entity_name)

    return LaunchDescription(launch_args + [
        OpaqueFunction(function=lambda context: launch_setup(context, robot_base_set, camera_set))
    ])

def launch_setup(context, robot_base_set, camera_set):
    config = launch_utils.get_common_launch_arguments(context, robot_base_set, camera_set)

    gz_pkg_share = get_package_share_directory('qrb_ros_sim_gazebo')
    robot_model_path = os.path.join(gz_pkg_share, 'urdf', 'gazebo_qrb_robot_base.urdf.xacro')

    # Launch the gazebo and load world model
    load_world_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': [os.path.join(gz_pkg_share, 'worlds', f'{config["world_model"]}.sdf') + ' -v 1']
        }.items()
    )

    robot_launch = generate_robot_launch(config, robot_model_path, robot_base_set, camera_set)

    return [load_world_model, *robot_launch]

def generate_robot_launch(config, robot_model_path, robot_base_set, camera_set):
    doc = xacro.process_file(
        robot_model_path,
        mappings={
            'robot_name': config['robot_entity_name'],
            'enable_laser': config['enable_laser'],
            'laser_config': config['laser_config_file'],
            'enable_imu': config['enable_imu'],
            'imu_config': config['imu_config_file'],
            'enable_odom_tf': config['enable_odom_tf'].lower(),
        }
    )

    # After starting the robot_state_publisher node, it will publish the robot_description topic,
    # which contains the content of the URDF model file. It will also subscribe to the /joint_states
    # topic to get joint data, and then publish the tf and tf_static topics.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"use_sim_time": True}, {'robot_description': doc.toxml()}],
        namespace=config['namespace'],
        output='screen'
    )

    # Create robot model in the gazebo world
    spawn_robot_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace=config['namespace'],
        arguments=[
            '-entity', config['robot_entity_name'],
            '-topic', 'robot_description',
            '-x', config['initial_x'],
            '-y', config['initial_y'],
            '-z', config['initial_z'],
            '-R', config['initial_roll'],
            '-P', config['initial_pitch'],
            '-Y', config['initial_yaw']
        ],
    )

    # Ros-Gazebo bridge
    ros_gz_bridges = []
    ros_gz_bridge_configs = launch_utils.get_ros_gz_bridge_configs(robot_base_set, camera_set, config)
    for bridge_config in ros_gz_bridge_configs:
        ros_gz_bridges.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=bridge_config['args'],
                namespace=config['namespace'],
                **({'remappings': bridge_config['remappings']} if 'remappings' in bridge_config else {}),
                output='screen'
            )
        )

    return [
        robot_state_publisher_node,
        spawn_robot_entity,
        *ros_gz_bridges,
    ]