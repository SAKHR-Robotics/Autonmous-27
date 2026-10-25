"""Production orchestration only: no camera drivers, synthetic inputs or fake TF."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,GroupAction,IncludeLaunchDescription,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def setup(context):
    def value(name):return LaunchConfiguration(name).perform(context)
    def enabled(name):return value(name)=='true'
    if enabled('launch_memory') and not enabled('launch_terrain'):
        raise ValueError('Memory requires terrain observations in this master; use its standalone launch for an external producer')
    def include(package,filename,arguments):
        return GroupAction(scoped=True,actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package),'launch',filename)),
            launch_arguments=arguments.items())])
    nodes=[]
    clock={'use_sim_time':value('use_sim_time')}
    if enabled('launch_terrain'):
        arguments={**clock,**{key:value(key) for key in ('input_pointcloud_topic','target_frame','enable_costmap',
            'enable_terrain_risk','gravity_frame','max_slope_deg','max_step_m','max_roughness_m')},
            'publish_to_obstacles_only':'false' if enabled('launch_memory') else 'true',
            'publish_debug_topics':value('publish_debug_topics')}
        nodes.append(include('terrain_geometry','terrain.launch.py',arguments))
    if enabled('launch_memory'):
        nodes.append(include('terrain_geometry','obstacle_memory.launch.py',{**clock,'params_file':value('memory_params_file')}))
    if enabled('launch_marker_branch'):
        arguments={**clock,**{key:value(key) for key in ('rgb_topic','depth_topic','camera_info_topic',
            'launch_marker_detection','launch_marker_tf','launch_marker_mapping','launch_marker_action','launch_landmark_adapter',
            'detection_params_file','tf_params_file','mapping_params_file','action_params_file','landmark_params_file')}}
        nodes.append(include('marker_detection','marker_branch.launch.py',arguments))
    if enabled('launch_rviz'):
        nodes.append(include('terrain_geometry','rviz.launch.py',{**clock,
            'rviz_config':value('rviz_config'),'fixed_frame':value('rviz_fixed_frame'),
            'rgb_topic':value('rgb_topic'),'input_pointcloud_topic':value('input_pointcloud_topic')}))
    return nodes


def generate_launch_description():
    marker=get_package_share_directory('marker_detection')
    terrain=get_package_share_directory('terrain_geometry')
    defaults={'use_sim_time':'true','launch_terrain':'true','launch_marker_branch':'true',
        'launch_marker_detection':'true','launch_marker_tf':'true','launch_marker_mapping':'false',
        'launch_marker_action':'true','launch_landmark_adapter':'false','launch_memory':'false',
        'launch_rviz':'false','enable_costmap':'false','enable_terrain_risk':'false','publish_debug_topics':'false',
        'target_frame':'base_link','rviz_fixed_frame':'base_link','gravity_frame':'odom',
        'max_slope_deg':'0.0','max_step_m':'0.0','max_roughness_m':'0.0',
        'input_pointcloud_topic':'/camera/depth/color/points','rgb_topic':'/camera/image_raw',
        'depth_topic':'/camera/depth/image_raw','camera_info_topic':'/camera/camera_info',
        'rviz_config':os.path.join(terrain,'rviz','perception_system_view.rviz'),
        'memory_params_file':os.path.join(terrain,'config','obstacle_memory.yaml')}
    for stage,stem in [('detection','marker_detection'),('tf','marker_tf'),('mapping','marker_mapping'),('action','marker_action_interface'),('landmark','landmark_adapter')]:
        defaults[stage+'_params_file']=os.path.join(marker,'config',stem+'.yaml')
    actions=[]
    for key,default in defaults.items():
        choices=['true','false'] if key.startswith(('launch_','enable_','publish_')) or key=='use_sim_time' else None
        actions.append(DeclareLaunchArgument(key,default_value=default,choices=choices))
    return LaunchDescription(actions+[OpaqueFunction(function=setup)])
