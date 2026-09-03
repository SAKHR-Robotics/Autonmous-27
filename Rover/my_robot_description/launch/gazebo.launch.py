#!/usr/bin/env python3
"""
gazebo.launch.py - Gazebo simulation launch file

This launch file:
1. Processes the xacro file into URDF
2. Launches Gazebo Ignition with empty world
3. Launches robot_state_publisher with use_sim_time=true
4. Spawns robot at origin using spawn_entity service

Use this to test the robot in Gazebo simulation.
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import re

def launch_setup(context, *args, **kwargs):
    # Retrieve the world launch argument
    world_file = LaunchConfiguration('world').perform(context)
    
    # Get package shares
    pkg_share = FindPackageShare('my_robot_description').find('my_robot_description')
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_worlds = get_package_share_directory('worlds')
    except Exception:
        pkg_worlds = ''
    
    # Resolve world path
    if os.path.isabs(world_file):
        world_path = world_file
    elif pkg_worlds and os.path.exists(os.path.join(pkg_worlds, 'worlds', world_file)):
        world_path = os.path.join(pkg_worlds, 'worlds', world_file)
    else:
        world_path = os.path.join(pkg_share, 'worlds', world_file)
        
    # Determine the world name inside the SDF/World file
    world_name = "rover_world"
    if "empty" in world_file:
        world_name = "empty"
    elif os.path.exists(world_path):
        try:
            with open(world_path, 'r') as f:
                content = f.read()
                match = re.search(r'<world\s+name=["\']([^"\']+)["\']>', content)
                if match:
                    world_name = match.group(1)
        except Exception as e:
            print(f"[gazebo.launch] Error reading world file: {e}")

    # Set up Gazebo resource paths to find marsyard and rock models
    resource_paths = [
        pkg_share,
        os.path.join(pkg_share, 'worlds'),
        os.path.join(pkg_share, '..'),  # to resolve package://my_robot_description
    ]

    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_worlds = get_package_share_directory('worlds')
        resource_paths.append(os.path.join(pkg_worlds, 'worlds'))
        resource_paths.append(pkg_worlds)
        resource_paths.append(os.path.dirname(pkg_worlds))
        resource_paths.append(os.path.join(pkg_worlds, 'models'))
        resource_paths.append(os.path.join(pkg_worlds, 'models', 'rocks'))
        resource_paths.append(os.path.join(pkg_worlds, 'models', 'aruco'))
    except Exception as e:
        print(f"[gazebo.launch] Share paths resolution: {e}")
    
    # Also add source tree models path as fallback
    workspace_models = os.path.abspath(os.path.join(pkg_share, '..', '..', 'worlds', 'models'))
    if os.path.exists(workspace_models):
        resource_paths.append(workspace_models)
        resource_paths.append(os.path.join(workspace_models, 'rocks'))
        resource_paths.append(os.path.join(workspace_models, 'aruco'))

    ign_existing = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    gz_existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    ign_path = os.pathsep.join(resource_paths)
    if ign_existing:
        ign_path = ign_path + os.pathsep + ign_existing
        
    gz_path = os.pathsep.join(resource_paths)
    if gz_existing:
        gz_path = gz_path + os.pathsep + gz_existing
        
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = ign_path
    os.environ['GZ_SIM_RESOURCE_PATH'] = gz_path

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
    
    # Gazebo Launch using standard ros_gz_sim package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(FindPackageShare('ros_gz_sim').find('ros_gz_sim'),
                        'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f"-r {world_path}",
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Spawn Robot Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-string', robot_description_config.toxml(),
            '-world', world_name,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.5'  # Spawn slightly above the ground to land gently without tunneling
        ],
        output='screen'
    )
    
    # Bridge between Ignition Gazebo and ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            f'/world/{world_name}/model/my_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            (f'/world/{world_name}/model/my_robot/joint_state', '/joint_states'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/image', '/camera/image_raw'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/camera_info', '/camera/camera_info'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/depth_image', '/camera/depth/image_raw'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/points', '/camera/depth/color/points'),
        ],
        output='screen'
    )
    return [
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        bridge
    ]


def generate_launch_description():
    # Declare the world argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='world1.world',
        description='Name of the world file or path to load (e.g., world1.world or empty_with_sensors.sdf)'
    )
    
    return LaunchDescription([
        world_arg,
        OpaqueFunction(function=launch_setup)
    ])

