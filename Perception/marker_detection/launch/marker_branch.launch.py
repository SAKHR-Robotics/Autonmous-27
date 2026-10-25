"""Independently selectable real marker stages; sources and localization external."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share=get_package_share_directory('marker_detection')
    actions=[DeclareLaunchArgument('use_sim_time',default_value='true')]
    for key,default in [('rgb_topic','/camera/image_raw'),('depth_topic','/camera/depth/image_raw'),('camera_info_topic','/camera/camera_info')]:
        actions.append(DeclareLaunchArgument(key,default_value=default))
    groups=[]
    for stage,stem,switch,default in [
        ('detection','marker_detection','launch_marker_detection','true'),
        ('tf','marker_tf','launch_marker_tf','true'),
        ('mapping','marker_mapping','launch_marker_mapping','true'),
        ('action','marker_action_interface','launch_marker_action','true'),
        ('landmark','landmark_adapter','launch_landmark_adapter','false')]:
        actions.append(DeclareLaunchArgument(switch,default_value=default,choices=['true','false']))
        forwarded={'use_sim_time':LaunchConfiguration('use_sim_time')}
        if stage in ('detection','tf','mapping','action','landmark'):
            name=stage+'_params_file'
            actions.append(DeclareLaunchArgument(name,default_value=os.path.join(share,'config',stem+'.yaml')))
            forwarded['params_file']=LaunchConfiguration(name)
        if stage=='detection':
            forwarded.update({name:LaunchConfiguration(name) for name in ('rgb_topic','depth_topic','camera_info_topic')})
        groups.append(GroupAction(scoped=True,condition=IfCondition(LaunchConfiguration(switch)),actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch',stem+'.launch.py')),
                launch_arguments=forwarded.items())]))
    return LaunchDescription(actions+groups)
