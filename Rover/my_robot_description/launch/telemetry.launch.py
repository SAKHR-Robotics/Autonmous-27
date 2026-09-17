#!/usr/bin/env python3
"""
telemetry.launch.py
===================
Launch file for the Rover Telemetry & Diagnostics Visualizer GUI,
along with the encoder_ticks_to_odom node for real-time 4-wheel telemetry.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    # Encoder ticks to odometry node
    encoder_node = Node(
        package='rover_slam',
        executable='encoder_ticks_to_odom',
        name='encoder_ticks_to_odom',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'track_width': 0.49,
            'wheel_names': ['left_front', 'right_front', 'left_rear', 'right_rear'],
        }]
    )

    # Telemetry Dashboard GUI node
    dashboard_node = Node(
        package='my_robot_description',
        executable='telemetry_dashboard.py',
        name='rover_telemetry_dashboard',
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        encoder_node,
        dashboard_node
    ])
