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
        DeclareLaunchArgument('params_file',default_value=os.path.join(get_package_share_directory('marker_detection'),'config','landmark_adapter.yaml')),
        Node(package='marker_detection',executable='marker_landmark_adapter',name='marker_landmark_adapter',output='screen',
            parameters=[LaunchConfiguration('params_file'),{'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}])])
