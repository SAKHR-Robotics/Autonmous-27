"""Optional physical camera driver; shared by Perception and SLAM."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=get_package_share_directory('rover_slam')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('params_file',default_value=os.path.join(share,'config','realsense_filters.yaml')),
        Node(package='realsense2_camera', executable='realsense2_camera_node',
            name='camera',namespace='camera',output='screen',
            parameters=[LaunchConfiguration('params_file'),
                {'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}],
            remappings=[('~/color/image_raw','/camera/image_raw'),
                ('~/color/camera_info','/camera/camera_info'),
                ('~/aligned_depth_to_color/image_raw','/camera/depth/image_raw'),
                ('~/depth/color/points','/camera/depth/color/points')])])
