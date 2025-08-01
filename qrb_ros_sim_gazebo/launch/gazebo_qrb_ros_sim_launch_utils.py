# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear
import os
import yaml
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def declare_common_launch_arguments(robot_base_set=False, camera_set=False,
    robot_entity_name='qrb_robot', init_pose=['0','0','0','0','0','0']):
    robot_gazebo_pkg = 'qrb_ros_sim_gazebo'
    default_laser_config = os.path.join(
        get_package_share_directory(robot_gazebo_pkg), 'config', 'params', 'qrb_robot_base_laser_params.yaml')
    default_imu_config = os.path.join(
        get_package_share_directory(robot_gazebo_pkg), 'config', 'params', 'imu_params.yaml')
    default_rgb_camera_config = os.path.join(
        get_package_share_directory(robot_gazebo_pkg), 'config', 'params', 'rgb_camera_params.yaml')
    default_depth_camera_config = os.path.join(
        get_package_share_directory(robot_gazebo_pkg), 'config', 'params', 'depth_camera_params.yaml')

    base_params = [
        DeclareLaunchArgument(
            'launch_config_file',
            default_value='',
            description=(
                'Path to a YAML configuration file. '
                'Parameters defined in this file have HIGHEST PRIORITY '
                'and will override the command-line and default values'
            )
        ),
        DeclareLaunchArgument(
            'world_model',
            default_value='warehouse',
            description='Name of the Gazebo world model to load (without .sdf extension)'
        ),
        DeclareLaunchArgument(
            'robot_entity_name',
            default_value=robot_entity_name,
            description='Name of the robot entity in the simulation environment'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='ROS namespace'
        ),
        DeclareLaunchArgument(
            'initial_x',
            default_value=init_pose[0],
            description='Initial X position (meters) in world coordinates'
        ),
        DeclareLaunchArgument(
            'initial_y',
            default_value=init_pose[1],
            description='Initial Y position (meters) in world coordinates'
        ),
        DeclareLaunchArgument(
            'initial_z',
            default_value=init_pose[2],
            description='Initial Z position (meters) in world coordinates'
        ),
        DeclareLaunchArgument(
            'initial_roll',
            default_value=init_pose[3],
            description='Initial roll orientation (radians) around X-axis'
        ),
        DeclareLaunchArgument(
            'initial_pitch',
            default_value=init_pose[4],
            description='Initial pitch orientation (radians) around Y-axis'
        ),
        DeclareLaunchArgument(
            'initial_yaw',
            default_value=init_pose[5],
            description='Initial yaw orientation (radians) around Z-axis'
        )
	]

    robot_base_params = [
	    DeclareLaunchArgument(
            'enable_laser',
            default_value='true',
            description='Enable/disable LiDAR sensor ("true"/"false")'
        ),
        DeclareLaunchArgument(
            'laser_config_file',
            default_value=default_laser_config,
            description='Path to LiDAR sensor configuration YAML file'
        ),
        DeclareLaunchArgument(
            'enable_imu',
            default_value='true',
            description='Enable/disable Inertial Measurement Unit (IMU) sensor ("true"/"false")'
        ),
        DeclareLaunchArgument(
            'imu_config_file',
            default_value=default_imu_config,
            description='Path to IMU configuration YAML file'
        ),
        DeclareLaunchArgument(
            'enable_odom',
            default_value='true',
            description='Enable/disable odometry data publication ("true"/"false")'
        ),
	]

    camera_params = [
	    DeclareLaunchArgument(
            'enable_rgb_camera',
            default_value='true',
            description='Enable/disable RGB camera sensor ("true"/"false")'
        ),
        DeclareLaunchArgument(
            'rgb_camera_config_file',
            default_value=default_rgb_camera_config,
            description='Path to RGB camera configuration YAML file'
        ),
        DeclareLaunchArgument(
            'enable_depth_camera',
            default_value='true',
            description='Enable/disable depth camera sensor ("true"/"false")'
        ),
        DeclareLaunchArgument(
            'depth_camera_config_file',
            default_value=default_depth_camera_config,
            description='Path to depth camera configuration YAML file'
        ),
	]

    return_params = base_params
    if robot_base_set:
	    return_params += robot_base_params
    if camera_set:
	    return_params += camera_params

    return return_params

def get_common_launch_arguments(context, robot_base_set=False, camera_set=False):
    base_launch_config = {
        'launch_config_file': LaunchConfiguration('launch_config_file').perform(context),
        'world_model': LaunchConfiguration('world_model').perform(context),
        'robot_entity_name': LaunchConfiguration('robot_entity_name').perform(context),
        'namespace': LaunchConfiguration('namespace').perform(context),
        'initial_x': LaunchConfiguration('initial_x').perform(context),
        'initial_y': LaunchConfiguration('initial_y').perform(context),
        'initial_z': LaunchConfiguration('initial_z').perform(context),
        'initial_roll': LaunchConfiguration('initial_roll').perform(context),
        'initial_pitch': LaunchConfiguration('initial_pitch').perform(context),
        'initial_yaw': LaunchConfiguration('initial_yaw').perform(context),
    }

    return_launch_config = base_launch_config
    if robot_base_set:
        robot_base_launch_config = {
            'enable_laser': LaunchConfiguration('enable_laser').perform(context),
            'laser_config_file': LaunchConfiguration('laser_config_file').perform(context),
            'enable_imu': LaunchConfiguration('enable_imu').perform(context),
            'imu_config_file': LaunchConfiguration('imu_config_file').perform(context),
            'enable_odom': LaunchConfiguration('enable_odom').perform(context),
        }
        return_launch_config |= robot_base_launch_config
    if camera_set:
        camera_launch_config = {
            'enable_rgb_camera': LaunchConfiguration('enable_rgb_camera').perform(context),
            'rgb_camera_config_file': LaunchConfiguration('rgb_camera_config_file').perform(context),
            'enable_depth_camera': LaunchConfiguration('enable_depth_camera').perform(context),
            'depth_camera_config_file': LaunchConfiguration('depth_camera_config_file').perform(context),
        }
        return_launch_config |= camera_launch_config

    if return_launch_config['launch_config_file']:
        with open(return_launch_config['launch_config_file'], 'r') as file:
            yaml_config = yaml.safe_load(file)
            for key in return_launch_config:
                if key in yaml_config:
                    return_launch_config[key] = yaml_config[key]

    return return_launch_config

def get_ros_gz_bridge_configs(robot_base_set=False, camera_set=False, launch_config={}):
    ros_gz_bridge_configs = []
    ## Clock
    ros_gz_bridge_configs.append(
        {'args': ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']}
    )
    if robot_base_set:
        ## Robot state
        ros_gz_bridge_configs.append(
            {'args': [
                'joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ]}
        )
        ## Differential driver
        ros_gz_bridge_configs.append(
            {'args': [
                'cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
            ]}
        )
        ## Odom
        if launch_config['enable_odom'].lower() == 'true':
            ros_gz_bridge_configs.append(
                {'args': [
                    'odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                ]}
            )
        ## Lidar
        if launch_config['enable_laser'].lower() == 'true':
            ros_gz_bridge_configs.append(
                {'args': ['scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan']}
            )
        ## IMU
        if launch_config['enable_imu'].lower() == 'true':
            ros_gz_bridge_configs.append(
                {'args': ['imu@sensor_msgs/msg/Imu[gz.msgs.IMU']}
            )
    if camera_set:
        ## RGB camera
        if launch_config['enable_rgb_camera'].lower() == 'true':
            ros_gz_bridge_configs.append(
                {'args': [
                    'camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                    'camera/color/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
                ]}
            )
        ## Depth camera
        if launch_config['enable_depth_camera'].lower() == 'true':
            ros_gz_bridge_configs.append({
                'args': [
                    'camera/depth/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                    'camera/depth@sensor_msgs/msg/Image@gz.msgs.Image',
                    'camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
                ],
                'remappings': [
                    ('camera/depth','camera/depth/image_raw'),
                    ('camera/camera_info','camera/depth/camera_info')
                ]
            })

    return ros_gz_bridge_configs