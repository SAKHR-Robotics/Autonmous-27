from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('input_topic',default_value='/perception/obstacles_only'),
        DeclareLaunchArgument('output_topic',default_value='/bridge/pointcloud'),
        Node(package='erc_path_planner',executable='costmap_bridge_node',name='costmap_bridge_node',
            parameters=[{'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool),
                         'input_topic':LaunchConfiguration('input_topic'),'output_topic':LaunchConfiguration('output_topic')}])])
