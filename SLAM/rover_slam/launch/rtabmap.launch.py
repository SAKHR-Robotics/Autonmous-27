"""RGB-D mapping with covariance-bearing local odometry and native landmarks."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(key): return LaunchConfiguration(key).perform(context)
    path = os.path.expanduser(value('database_path'))
    if not os.path.isabs(path):
        raise ValueError('database_path must be absolute (or begin with ~/); use a distinct path per test run')
    return [Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
        arguments=['-d'] if value('delete_db_on_start')=='true' else [],
        parameters=[value('params_file'), {'use_sim_time':value('use_sim_time')=='true',
            'database_path':path, 'odom_frame_id':'', 'subscribe_odom':True}],
        remappings=[('odom',value('odom_topic')), ('rgb/image',value('rgb_topic')),
            ('depth/image',value('depth_topic')), ('rgb/camera_info',value('camera_info_topic')),
            ('landmark_detections',value('landmarks_topic')),
            ('landmark_detection',value('landmark_topic')),
            # RTAB-Map versions use map or grid_map for their occupancy output.
            ('grid_map','/map'), ('map','/map')])]


def generate_launch_description():
    share=get_package_share_directory('rover_slam')
    defaults={'use_sim_time':'false','delete_db_on_start':'false',
        'params_file':os.path.join(share,'config','rtabmap.yaml'), 'database_path':'~/.ros/rtabmap.db',
        'odom_topic':'/odometry/filtered','rgb_topic':'/camera/image_raw',
        'depth_topic':'/camera/depth/image_raw','camera_info_topic':'/camera/camera_info',
        'landmarks_topic':'/perception/landmark_detections','landmark_topic':'/perception/landmark_detection'}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v,
        choices=['true','false'] if k in ('use_sim_time','delete_db_on_start') else None)
        for k,v in defaults.items()]+[OpaqueFunction(function=setup)])
