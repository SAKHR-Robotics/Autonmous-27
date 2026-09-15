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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
import re

def launch_setup(context, *args, **kwargs):
    # World name in Gazebo (marsyard by default in world1.world and final_world_RA.world)
    world_arg = LaunchConfiguration('world').perform(context)
    world_name = world_arg.replace('.world', '') if world_arg else 'marsyard'

    # Get my_robot_description package share directory
    pkg_share = FindPackageShare('my_robot_description').find('my_robot_description')
    
    # Path to the xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    
    # Process the xacro file to generate URDF
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_config.toxml()

    # Apply ROS REP-103 standard camera optical frames and rotation (Task: SLAM Fix 4)
    # Optical convention: +Z forward, +X right, +Y down (rpy="-1.57079632679 0 -1.57079632679")
    optical_rep103_frames = """
  <!-- Standard ROS REP-103 Optical Frames (Task: SLAM Fix 4) -->
  <link name="camera_depth_optical_frame"/>
  <joint name="camera_depth_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_depth_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.57079632679 0 -1.57079632679"/>
  </joint>
  <link name="camera_color_optical_frame"/>
  <joint name="camera_color_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_color_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.57079632679 0 -1.57079632679"/>
  </joint>
"""
    robot_description_xml = re.sub(
        r'(<joint name="camera_optical_joint"[^>]*>.*?<origin\s+[^>]*?)rpy="[^"]*"',
        r'\1rpy="-1.57079632679 0 -1.57079632679"',
        robot_description_xml,
        flags=re.DOTALL
    )
    if '<link name="camera_depth_optical_frame"' not in robot_description_xml:
        robot_description_xml = robot_description_xml.replace('</robot>', optical_rep103_frames + '\n</robot>')

    robot_description = {'robot_description': robot_description_xml}
    
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
    
    # The world name is marsyard as defined in world1.world
    world_name = "marsyard"
    
    # Spawn Robot Entity (Default height z:=1.5 to land gently)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-string', robot_description_xml,
            '-world', world_name,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.5'
        ],
        output='screen'
    )
    
    # Bridge between Gazebo and ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            f'/world/{world_name}/model/my_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/imu', '/imu/data'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/image', '/camera/image_raw'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/camera_info', '/camera/camera_info'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/depth_image', '/camera/depth/image_raw'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/points', '/camera/depth/color/points'),
            (f'/world/{world_name}/model/my_robot/joint_state', '/joint_states_gz'),
        ],
        output='screen'
    )

    # Joint state publisher to ensure continuous wheel joints are published to TF
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'source_list': ['/joint_states_gz']
        }],
        output='screen'
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
        bridge
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='marsyard',
            description='World name in Gazebo simulation (e.g. marsyard, empty)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
