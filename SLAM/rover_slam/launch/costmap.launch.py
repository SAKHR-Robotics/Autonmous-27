import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_slam')
    costmap_config_path = os.path.join(pkg_share, 'config', 'costmap_params.yaml')

    return LaunchDescription([
        # Nav2 Costmap 2D Lifecycle Node
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            output='screen',
            parameters=[costmap_config_path]
        ),

        # Nav2 Lifecycle Manager to automatically configure and activate global_costmap
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='costmap_lifecycle_manager',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'node_names': ['global_costmap']}
            ]
        )
    ])

