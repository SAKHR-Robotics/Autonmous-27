import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('fixed_frame',default_value='base_link'),
        DeclareLaunchArgument('rgb_topic',default_value='/camera/image_raw'),
        DeclareLaunchArgument('input_pointcloud_topic',default_value='/camera/depth/color/points'),
        DeclareLaunchArgument('rviz_config',default_value=os.path.join(get_package_share_directory('marker_detection'),'rviz','marker_detection_view.rviz')),
        Node(package='rviz2',executable='rviz2',name='marker_detection_rviz',
            arguments=['-d',LaunchConfiguration('rviz_config'),'-f',LaunchConfiguration('fixed_frame')],
            parameters=[{'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}],
            remappings=[('/camera/image_raw',LaunchConfiguration('rgb_topic')),
                        ('/camera/depth/color/points',LaunchConfiguration('input_pointcloud_topic'))])])
