#!/usr/bin/env python3
"""
spawn_rover.launch.py - Spawns the rover model and sets up bridges

This launch file:
1. Processes the xacro file into URDF
2. Launches robot_state_publisher with use_sim_time=true
3. Spawns the robot using spawn_entity (ros_gz_sim create)
4. Configures the ROS-Gazebo parameter bridges
(Assumes Gazebo is already running, e.g. via world1.launch.py)
"""

import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import re

def launch_setup(context, *args, **kwargs):
    # Get my_robot_description package share directory
    pkg_share = FindPackageShare('my_robot_description').find('my_robot_description')
    
    # Path to the xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    
    # Process the xacro file to generate URDF
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ]
    )
    
    # Spawn Robot Entity (Default height z:=1.5 to land gently)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.5'
        ],
        output='screen'
    )
    
    # Bridge between Gazebo and ROS 2
    # The world name is marsyard as defined in world1.world
    world_name = "marsyard"
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/image', '/camera/image_raw'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/camera_info', '/camera/camera_info'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/depth_image', '/camera/depth/image_raw'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/points', '/camera/depth/color/points'),
        ],
        output='screen'
    )
    
    return [
        robot_state_publisher_node,
        spawn_entity,
        bridge
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
