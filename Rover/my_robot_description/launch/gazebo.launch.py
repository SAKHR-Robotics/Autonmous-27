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
            '-string', robot_description_xml,
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
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            f'/world/{world_name}/model/my_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            f'/model/my_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/imu', '/imu/data'),
            (f'/world/{world_name}/model/my_robot/joint_state', '/joint_states_gz'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/image', '/camera/image_raw'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/camera_info', '/camera/camera_info'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/depth_image', '/camera/depth/image_raw'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/points', '/camera/depth/color/points'),
            (f'/model/my_robot/tf', '/tf'),
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

    publish_map_tf = LaunchConfiguration('publish_map_tf').perform(context).lower() in ['true', '1']
    publish_camera_tf = LaunchConfiguration('publish_camera_tf').perform(context).lower() in ['true', '1']

    # Static transform publisher to bridge Gazebo's camera sensor frame to REP-103 optical frame
    # (Only active in standalone mode when SLAM static_transforms.launch.py is NOT running)
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_bridge',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'camera_depth_optical_frame',
                   '--child-frame-id', 'my_robot/camera_link/camera'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Static transform publisher to bridge map to odom
    # (Only active for standalone teleop when SLAM RTAB-Map is NOT running)
    map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_bridge',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map',
                   '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    nodes = [
        gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
        bridge,
    ]
    if publish_camera_tf:
        nodes.append(camera_tf_node)
    if publish_map_tf:
        nodes.append(map_to_odom_node)

    return nodes


def generate_launch_description():
    # Declare the world argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='world1.world',
        description='Name of the world file or path to load (e.g., world1.world or empty_with_sensors.sdf)'
    )
    publish_map_tf_arg = DeclareLaunchArgument(
        'publish_map_tf',
        default_value='false',
        description='Publish static map -> odom transform (set false when SLAM RTAB-Map is active)'
    )
    publish_camera_tf_arg = DeclareLaunchArgument(
        'publish_camera_tf',
        default_value='false',
        description='Publish static camera optical TF (set false when SLAM static_transforms.launch.py is active)'
    )

    return LaunchDescription([
        world_arg,
        publish_map_tf_arg,
        publish_camera_tf_arg,
        OpaqueFunction(function=launch_setup)
    ])

