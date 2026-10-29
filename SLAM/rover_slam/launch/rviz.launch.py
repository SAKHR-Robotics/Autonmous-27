import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(key): return LaunchConfiguration(key).perform(context)
    view=value('rviz_view')
    filename='slam_local.rviz' if view=='local' else 'slam_visualization.rviz'
    config=value('rviz_config') or os.path.join(get_package_share_directory('rover_slam'),'config',filename)
    return [Node(package='rviz2',executable='rviz2',name='slam_rviz',
        arguments=['-d',config,'-f','odom' if view=='local' else 'map'],
        parameters=[{'use_sim_time':value('use_sim_time')=='true'}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='false',choices=['true','false']),
        DeclareLaunchArgument('rviz_view',default_value='global',choices=['local','global']),
        DeclareLaunchArgument('rviz_config',default_value=''),
        OpaqueFunction(function=setup)])
