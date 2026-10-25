import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    clock=LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('launch_aruco_stub',default_value='false'),
        DeclareLaunchArgument('launch_costmap_stub',default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rover_slam'),'launch','slam_bringup.launch.py')),
            launch_arguments={'use_sim_time':clock}.items()),
        Node(package='slam_test_tools',executable='mock_aruco_publisher',condition=IfCondition(LaunchConfiguration('launch_aruco_stub')),
            parameters=[{'use_sim_time':ParameterValue(clock,value_type=bool)}]),
        Node(package='slam_test_tools',executable='costmap_test_stub',condition=IfCondition(LaunchConfiguration('launch_costmap_stub')),
            parameters=[{'use_sim_time':ParameterValue(clock,value_type=bool)}])])
