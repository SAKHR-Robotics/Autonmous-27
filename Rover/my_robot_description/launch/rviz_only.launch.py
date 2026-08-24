#!/usr/bin/env python3
"""
rviz_only.launch.py - RViz visualization without robot_state_publisher

Use this when robot_state_publisher is already running (e.g., from Gazebo simulation).
This only launches RViz2 for visualization.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get the package share directory
    pkg_share = FindPackageShare('my_robot_description').find('my_robot_description')
    
    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'robot_view.rviz')] if os.path.exists(os.path.join(pkg_share, 'rviz', 'robot_view.rviz')) else []
    )
    
    return LaunchDescription([
        rviz_node
    ])
