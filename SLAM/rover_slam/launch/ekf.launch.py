import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    def value(key): return LaunchConfiguration(key).perform(context)
    share = get_package_share_directory('rover_slam')
    actions = []
    def include(filename, arguments):
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', filename)),
            launch_arguments=arguments.items())])
    if value('launch_encoder') == 'true':
        if value('odom_topic') != '/wheel/odom_raw':
            raise ValueError('launch_encoder requires odom_topic:=/wheel/odom_raw; disable it for external odometry')
        actions.append(include('encoder.launch.py', {'use_sim_time': value('use_sim_time'),
            'params_file': value('encoder_params_file'), 'input_mode': value('encoder_input_mode')}))
    if value('launch_slip') == 'true':
        actions.append(include('slip.launch.py', {'use_sim_time': value('use_sim_time'),
            'params_file': value('slip_params_file'), 'odom_topic': value('odom_topic'), 'imu_topic': value('imu_topic')}))
    actions.append(Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen', parameters=[value('ekf_params_file'), {
            'use_sim_time': value('use_sim_time') == 'true',
            'odom0': '/wheel/odom_filtered' if value('launch_slip') == 'true' else value('odom_topic'),
            'imu0': value('imu_topic')}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory('rover_slam')
    defaults = {'use_sim_time': 'false', 'launch_encoder': 'false', 'launch_slip': 'true',
        'odom_topic': '/wheel/odom_raw', 'imu_topic': '/imu/data', 'encoder_input_mode': 'joint_states'}
    defaults.update({key+'_params_file': os.path.join(share, 'config', key+'.yaml')
                     for key in ('encoder', 'slip', 'ekf')})
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v,
        choices=['true', 'false'] if k.startswith('launch_') or k=='use_sim_time' else None)
        for k,v in defaults.items()] + [OpaqueFunction(function=setup)])
