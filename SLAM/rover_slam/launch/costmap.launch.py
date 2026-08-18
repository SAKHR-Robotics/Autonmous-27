import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_slam')
    costmap_config_path = os.path.join(pkg_share, 'config', 'costmap_params.yaml')

    return LaunchDescription([
        # Static TF Publisher (map -> base_link) for standalone testing
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='costmap_test_tf_map_to_base',
            arguments=['--frame-id', 'map', '--child-frame-id', 'base_link']
        ),

        # Nav2 Costmap 2D Lifecycle Node
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='costmap',
            output='screen',
            parameters=[costmap_config_path],
            remappings=[
                ('/costmap/costmap', '/global_costmap/costmap'),
                ('/costmap/costmap_updates', '/global_costmap/costmap_updates'),
            ]
        ),

        # Nav2 Lifecycle Manager to automatically configure and activate costmap
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='costmap_lifecycle_manager',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'bond_timeout': 0.0},
                {'node_names': ['costmap/costmap']}
            ]
        )
    ])
