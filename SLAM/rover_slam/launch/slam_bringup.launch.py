import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Production orchestration: test fixtures and artificial localization live in testing/.
def setup(context):
    def value(key): return LaunchConfiguration(key).perform(context)
    share = get_package_share_directory('rover_slam')
    actions = []
    def include(filename, keys, extra=None):
        arguments = {'use_sim_time': value('use_sim_time'), **{k: value(k) for k in keys}}
        arguments.update(extra or {})
        return GroupAction(scoped=True, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', filename)),
            launch_arguments=arguments.items())])
    if value('launch_ekf') == 'true':
        actions.append(include('ekf.launch.py', ('launch_encoder', 'launch_slip', 'odom_topic',
            'imu_topic', 'encoder_input_mode', 'base_frame', 'odom_frame', 'map_frame', 'encoder_params_file', 'slip_params_file', 'ekf_params_file')))
    elif value('launch_encoder') == 'true':
        raise ValueError('Master launch_encoder requires launch_ekf; use encoder.launch.py independently')
    if value('launch_rtabmap') == 'true':
        actions.append(include('rtabmap.launch.py', ('rgb_topic', 'depth_topic', 'camera_info_topic',
            'database_path', 'delete_db_on_start', 'landmarks_topic', 'base_frame', 'map_frame'),
            {'params_file': value('rtabmap_params_file'), 'odom_topic': '/odometry/filtered'}))
    if value('launch_static_tf') == 'true':
        actions.append(include('static_transforms.launch.py', ('publish_optical_tf', 'publish_gazebo_optical_tf'),
                               {'standalone': value('standalone_tf'), 'base_frame':value('base_frame')}))
    elif value('publish_gazebo_optical_tf') == 'true' or value('publish_optical_tf') == 'true' or value('standalone_tf') == 'true':
        raise ValueError('Static TF switches require launch_static_tf:=true')
    if value('launch_camera') == 'true':
        actions.append(include('vision_helper.launch.py', (), {'params_file': value('camera_params_file')}))
    if value('launch_costmap') == 'true':
        actions.append(include('costmap.launch.py', ('autostart', 'obstacle_topic', 'clearing_cloud_topic', 'base_frame', 'map_frame'),
                               {'params_file': value('costmap_params_file')}))
    if value('launch_rviz') == 'true':
        actions.append(include('rviz.launch.py', ('rviz_view',), {'rviz_config': value('rviz_config'), 'fixed_frame':value('rviz_fixed_frame') or
            (value('odom_frame') if value('rviz_view')=='local' else value('map_frame'))}))
    return actions


def generate_launch_description():
    share = get_package_share_directory('rover_slam')
    defaults = {'use_sim_time': 'false', 'launch_ekf': 'true', 'launch_rtabmap': 'true',
        'launch_encoder': 'false', 'launch_slip': 'true', 'launch_camera': 'false',
        'launch_static_tf': 'false', 'standalone_tf': 'false', 'publish_optical_tf': 'false',
        'publish_gazebo_optical_tf': 'false', 'launch_costmap': 'false', 'launch_rviz': 'false',
        'autostart': 'true', 'odom_topic': '/odom', 'imu_topic': '/imu/data',
        'encoder_input_mode': 'joint_states', 'rgb_topic': '/camera/image_raw',
        'depth_topic': '/camera/depth/image_raw', 'camera_info_topic': '/camera/camera_info',
        'landmarks_topic': '/perception/landmark_detections', 'database_path': '~/.ros/rtabmap.db',
        'delete_db_on_start': 'false', 'obstacle_topic': '/bridge/pointcloud',
        'clearing_cloud_topic': '/camera/depth/color/points', 'base_frame':'base_link',
        'odom_frame':'odom','map_frame':'map','rviz_fixed_frame':'', 'rviz_view': 'global', 'rviz_config': ''}
    for name, stem in [('encoder','encoder'), ('slip','slip'), ('ekf','ekf'), ('rtabmap','rtabmap'),
                       ('camera','realsense_filters'), ('costmap','costmap_params')]:
        defaults[name+'_params_file'] = os.path.join(share, 'config', stem+'.yaml')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v,
        choices=['true', 'false'] if k.startswith(('launch_', 'publish_')) or k in
        ('use_sim_time','standalone_tf','autostart','delete_db_on_start') else None)
        for k,v in defaults.items()] + [OpaqueFunction(function=setup)])
