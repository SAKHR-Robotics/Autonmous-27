#!/usr/bin/env python3
"""
gazebo_with_teleop.launch.py - Launches Gazebo simulation with rover and teleop GUI
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('my_robot_description').find('my_robot_description')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='world_Rotated_Aruco.world',
        description='World file or path to load in simulation'
    )

    publish_map_tf_arg = DeclareLaunchArgument(
        'publish_map_tf',
        default_value='true',
        description='Publish static map -> odom transform for standalone teleop visualization'
    )

    publish_camera_tf_arg = DeclareLaunchArgument(
        'publish_camera_tf',
        default_value='true',
        description='Publish static camera optical TF for standalone teleop visualization'
    )

    bridge_sim_tf_arg = DeclareLaunchArgument(
        'bridge_sim_tf',
        default_value='true',
        description='Bridge Gazebo simulation TF for standalone teleop visualization'
    )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'publish_map_tf': LaunchConfiguration('publish_map_tf'),
            'publish_camera_tf': LaunchConfiguration('publish_camera_tf'),
            'bridge_sim_tf': LaunchConfiguration('bridge_sim_tf'),
        }.items()
    )

    teleop_gui_node = Node(
        package='my_robot_description',
        executable='teleop_gui.py',
        name='rover_teleop_gui',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        publish_map_tf_arg,
        publish_camera_tf_arg,
        bridge_sim_tf_arg,
        gazebo_sim,
        teleop_gui_node
    ])
