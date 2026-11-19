import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Retrieve configuration paths
    pkg_share = get_package_share_directory('global_path_benchmarking')
    
    config_file = LaunchConfiguration('config').perform(context)
    bench_config_file = LaunchConfiguration('benchmark_config').perform(context)
    scenario_id = LaunchConfiguration('scenario_id').perform(context)
    verify_str = LaunchConfiguration('verify').perform(context).lower()
    clean_str = LaunchConfiguration('clean').perform(context).lower()
    use_rviz_str = LaunchConfiguration('use_rviz').perform(context).lower()
    
    # Resolve full path to config if it is relative
    if not os.path.isabs(config_file) and not os.path.exists(config_file):
        config_file = os.path.join(pkg_share, config_file)
        
    if not os.path.isabs(bench_config_file) and not os.path.exists(bench_config_file):
        bench_config_file = os.path.join(pkg_share, bench_config_file)
        
    # Build CLI arguments list for the testing node
    testing_args = ['--config', config_file, '--benchmark_config', bench_config_file]
    if scenario_id:
        testing_args.extend(['--scenario_id', scenario_id])
    if verify_str == 'true':
        testing_args.append('--verify')
    if clean_str == 'true':
        testing_args.append('--clean')
        
    # Testing Orchestrator Node
    testing_node = Node(
        package='global_path_benchmarking',
        executable='testing_node',
        name='testing_node',
        arguments=testing_args,
        output='screen'
    )
    
    nodes = [testing_node]

    if use_rviz_str == 'true':
        rviz_config = os.path.join(pkg_share, 'rviz', 'live_path_tracking.rviz')
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
        nodes.append(rviz_node)

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='config/scenarios.yaml',
            description='Path to the scenarios YAML file'
        ),
        DeclareLaunchArgument(
            'benchmark_config',
            default_value='config/benchmark_config.yaml',
            description='Path to the benchmark config YAML file'
        ),
        DeclareLaunchArgument(
            'scenario_id',
            default_value='',
            description='Specific scenario ID to run (runs all if empty)'
        ),
        DeclareLaunchArgument(
            'verify',
            default_value='false',
            description='Set to true to generate verification images and exit'
        ),
        DeclareLaunchArgument(
            'clean',
            default_value='false',
            description='Kill old benchmarking/ROS 2 processes before launch'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz2 to visualize scenario maps and moving rover live'
        ),
        OpaqueFunction(function=launch_setup)
    ])
