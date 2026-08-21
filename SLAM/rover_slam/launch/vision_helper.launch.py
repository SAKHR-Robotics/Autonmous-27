import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_slam')
    filters_config_path = os.path.join(pkg_share, 'config', 'realsense_filters.yaml')

    return LaunchDescription([
        # RealSense D435 Camera Driver with Depth Post-Processing Filters
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera',
            namespace='camera',
            output='screen',
            parameters=[filters_config_path],
            remappings=[
                ('depth/image_rect_raw', 'depth/filtered')
            ]
        )
    ])

