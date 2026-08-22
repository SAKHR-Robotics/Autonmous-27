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

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    teleop_gui_node = Node(
        package='my_robot_description',
        executable='teleop_gui.py',
        name='rover_teleop_gui',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gazebo_sim,
        teleop_gui_node
    ])
