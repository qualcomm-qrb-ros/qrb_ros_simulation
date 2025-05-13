# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear
import os
import yaml
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_gazebo_pkg = 'qrb_ros_sim_gazebo'
    world_pkg = 'qrb_ros_sim_description'

    gz_pkg_share = FindPackageShare(package=robot_gazebo_pkg).find(robot_gazebo_pkg) 
    world_pkg_share = FindPackageShare(package=world_pkg).find(world_pkg) 
    robot_model_path = os.path.join(gz_pkg_share,'urdf','gazebo_qrb_mobile_manipulator_robot.urdf.xacro')
    
    nodes = []
    launch_config_path = os.path.join(gz_pkg_share,'config','qrb_mobile_manipulator_launch_config.yaml')
    ros2_controller_yaml_path = os.path.join(gz_pkg_share,'config','params','ros2_controllers.yaml')
    with open(launch_config_path, 'r') as f:
        config = yaml.safe_load(f)
        world_file = f'{config['world_model']}.sdf'
        robot_group_args = config.get('robot_group_args', {})
        for robot_config in robot_group_args:
            nodes.extend(generate_robot_nodes(robot_config, robot_model_path, ros2_controller_yaml_path))

    # launch the gazebo and load world model
    load_world_model = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments=[
                ('gz_args', [os.path.join(world_pkg_share,'worlds',f'{world_file}'), ' -v 1'])
        ]
    )

    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    ld = LaunchDescription([
        load_world_model,
        *nodes,
        clock_bridge,
    ])
    return ld

def strip_ns_str(ns_str):
    return '/' + ns_str.strip('/')

def process_ros2_control_yaml(arm_ns, yaml_path):
    # Add namespace for joints and generate new {arm_ns}_ros2_controller.yaml file
    original_filename = os.path.basename(yaml_path)
    new_filename = f"{arm_ns}_{original_filename}"
    new_yaml_path = os.path.join(os.path.dirname(yaml_path), new_filename)

    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    for controller in config.values():
        params = controller.get('ros__parameters', {})
        if 'joints' in params:
            params['joints'] = [f"{arm_ns}/{joint}" for joint in params['joints']]
    # save new {arm_ns}_ros2_controller.yaml file
    with open(new_yaml_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)

def generate_robot_nodes(robot_config, robot_model_path, ros2_controller_yaml_path):
    robot_name = robot_config['robot_name']
    robot_base_ns = robot_config['robot_base_ns']
    arm_ns = robot_config['arm_ns']
    if arm_ns != '':
        process_ros2_control_yaml(arm_ns, ros2_controller_yaml_path)
    # Publish robot state
    enable_laser = robot_config['enable_laser']
    enable_imu = robot_config['enable_imu']
    enable_rgb_camera = robot_config['enable_rgb_camera']
    enable_depth_camera = robot_config['enable_depth_camera']
    doc = xacro.process_file(
        robot_model_path,
        mappings={
            'robot_name': robot_name,
            'robot_base_ns': robot_base_ns,
            'arm_ns': arm_ns,
            'enable_laser': enable_laser,
            'enable_imu': enable_imu,
            'enable_rgb_camera': enable_rgb_camera,
            'enable_depth_camera': enable_depth_camera,
            }
    )

    # After starting the robot_state_publisher node, it will publish the robot_description topic,
    # which contains the content of the URDF model file. It will also subscribe to the /joint_states
    # topic to get joint data, and then publish the tf and tf_static topics.
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"use_sim_time": True}, {'robot_description': doc.toxml()}],
        namespace=robot_base_ns,
        output='screen'
    )
    node_arm_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"use_sim_time": True}, {'robot_description': doc.toxml()}],
        namespace=robot_base_ns,
        output='screen',
        remappings=[
            (strip_ns_str(f"/{robot_base_ns}/joint_states"), strip_ns_str(f"/{arm_ns}/joint_states"))
        ],
    )

    # Create robot model in the gazebo world
    x = robot_config['initial_x']
    y = robot_config['initial_y']
    z = robot_config['initial_z']
    roll = robot_config['initial_roll']
    pitch = robot_config['initial_pitch']
    yaw = robot_config['initial_yaw']
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace=robot_base_ns,
        arguments=['-entity', robot_name, 
                    '-topic', 'robot_description',
                    '-x', x, '-y', y, '-z', z,
                    '-R', roll, '-P', pitch, '-Y', yaw],
    )

    # Bridge
    bridge_configs = [
        {   # depth camera
            'args': [
                f'{arm_ns}/camera/depth/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                f'{arm_ns}/camera/depth/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo']
        },
        {   # RGB camera
            'args': [
                f'{arm_ns}/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
                f'{arm_ns}/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo']
        },
        {   # lidar
            'args': [f'{robot_base_ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']
        },
        {   # IMU
            'args': [f'{robot_base_ns}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU']
        },
        {   # differential driver
            'args': [
                f'{robot_base_ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'{robot_base_ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist']
        },
        {   # robot state
            'args': [
                f'{robot_base_ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V']
        }
    ]
    bridges = []
    for config in bridge_configs:
        bridges.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=config['args'],
                output='screen'
            )
        )

    pub_world_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_odom",
        arguments=[ 
            "--frame-id", "world",
            "--child-frame-id", f'{robot_base_ns}/odom'.strip('/')
        ],
        output="screen"
    )

    return [
        node_robot_state_publisher,
        node_arm_state_publisher,
        gz_spawn_entity,
        *bridges,
        pub_world_odom_tf,
    ]
