"""Real SLAM against external Gazebo/recorded sensors; mocks are explicit only."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults={'use_sim_time':'true','launch_aruco_stub':'false','launch_costmap_stub':'false',
        'launch_rviz':'true','rviz_view':'global','launch_ekf':'true','launch_rtabmap':'true',
        'launch_encoder':'false','launch_slip':'true','odom_topic':'/odom','imu_topic':'/imu/data',
        'encoder_input_mode':'joint_states','rgb_topic':'/camera/image_raw',
        'depth_topic':'/camera/depth/image_raw','camera_info_topic':'/camera/camera_info',
        'database_path':'~/.ros/slam_standalone.db','delete_db_on_start':'false','launch_costmap':'false'}
    actions=[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]
    actions.append(GroupAction(scoped=True,actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rover_slam'),
        'launch','slam_bringup.launch.py')),launch_arguments={k:LaunchConfiguration(k) for k in defaults
        if k not in ('launch_aruco_stub','launch_costmap_stub')}.items())]))
    for flag,executable in [('launch_aruco_stub','mock_aruco_publisher'),('launch_costmap_stub','costmap_test_stub')]:
        actions.append(Node(package='slam_test_tools',executable=executable,condition=IfCondition(LaunchConfiguration(flag)),
            parameters=[{'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}]))
    return LaunchDescription(actions)
