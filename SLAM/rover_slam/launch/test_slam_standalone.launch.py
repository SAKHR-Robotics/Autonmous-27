#!/usr/bin/env python3
"""
test_slam_standalone.launch.py - Standalone Testing Launcher for SLAM Subsystem

Spins up rover_slam with mock ArUco landmark publisher and standalone Nav2
costmap enabled, plus RViz2 for visualization. Use this when testing the SLAM
stack in isolation without Perception or Path Planning active.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_slam')

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_aruco_stub = LaunchConfiguration('launch_aruco_stub')
    launch_costmap = LaunchConfiguration('launch_costmap')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_launch_rviz = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz2 visualization dashboard'
    )
    declare_launch_aruco_stub = DeclareLaunchArgument(
        'launch_aruco_stub',
        default_value='true',
        description='Whether to run mock_aruco_publisher for standalone SLAM landmark testing'
    )
    declare_launch_costmap = DeclareLaunchArgument(
        'launch_costmap',
        default_value='true',
        description='Whether to launch standalone Nav2 costmap for standalone verification'
    )

    slam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'slam_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'launch_rviz': launch_rviz,
            'launch_aruco_stub': launch_aruco_stub,
            'launch_costmap': launch_costmap,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_launch_rviz,
        declare_launch_aruco_stub,
        declare_launch_costmap,
        slam_bringup,
    ])
