"""Launch action-correlated PathPlanner/Controller batch benchmark runner."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('global_path_benchmarking')
    mock_share = get_package_share_directory('pathplanner_mock_tools')

    config_file = LaunchConfiguration('config').perform(context)
    bench_config_file = LaunchConfiguration('benchmark_config').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    scenario_id = LaunchConfiguration('scenario_id').perform(context)
    category = LaunchConfiguration('category').perform(context)
    timeout_s = LaunchConfiguration('timeout').perform(context)
    seed = LaunchConfiguration('seed').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context).lower()
    clean_str = LaunchConfiguration('clean').perform(context).lower()
    use_rviz_str = LaunchConfiguration('use_rviz').perform(context).lower()
    start_mocks_str = LaunchConfiguration('start_mocks').perform(context).lower()
    legacy_str = LaunchConfiguration('legacy').perform(context).lower()

    if clean_str == 'true':
        raise ValueError('--clean is disabled: stop only processes owned by your validation run')

    if not os.path.isabs(config_file) and not os.path.exists(config_file):
        config_file = os.path.join(pkg_share, config_file)
    if not os.path.isabs(bench_config_file) and not os.path.exists(bench_config_file):
        bench_config_file = os.path.join(pkg_share, bench_config_file)

    actions = []
    if start_mocks_str == 'true':
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mock_share, 'launch', 'mocked_planner.launch.py')
                ),
                launch_arguments={
                    'use_rviz': use_rviz_str,
                    'seed': seed,
                    'geometry_profile': LaunchConfiguration('geometry_profile').perform(context),
                    'clear_default_obstacles': 'true',
                }.items(),
            )
        )
    elif use_rviz_str == 'true':
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(pkg_share, 'rviz', 'live_path_tracking.rviz')],
                parameters=[{'use_sim_time': use_sim_time_str == 'true'}],
                output='screen',
            )
        )

    if legacy_str == 'true':
        testing_args = ['--config', config_file, '--benchmark_config', bench_config_file]
        if scenario_id:
            testing_args.extend(['--scenario_id', scenario_id])
        actions.append(
            Node(
                package='global_path_benchmarking',
                executable='testing_node',
                name='testing_node',
                arguments=testing_args,
                output='screen',
            )
        )
    else:
        runner_args = [
            '--output-dir', output_dir,
            '--mode', mode,
            '--scenarios', config_file,
            '--benchmark-config', bench_config_file,
            '--timeout', timeout_s,
            '--seed', seed,
        ]
        if scenario_id:
            runner_args.extend(['--scenario-id', scenario_id])
        if category:
            runner_args.extend(['--category', category])
        if use_sim_time_str == 'true':
            runner_args.append('--use-sim-time')
        if LaunchConfiguration('mocked_rover').perform(context).lower() == 'false':
            runner_args.append('--no-mocked-rover')

        actions.append(
            Node(
                package='global_path_benchmarking',
                executable='scenario_benchmark_runner',
                name='scenario_benchmark_runner',
                arguments=runner_args,
                output='screen',
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='/tmp/pathplanner_benchmark'),
        DeclareLaunchArgument('mode', default_value='nav_closed_loop',
                              choices=['planner_only', 'nav_closed_loop', 'mixed']),
        DeclareLaunchArgument('config', default_value='config/scenarios.yaml'),
        DeclareLaunchArgument('benchmark_config', default_value='config/benchmark_config.yaml'),
        DeclareLaunchArgument('scenario_id', default_value=''),
        DeclareLaunchArgument('category', default_value=''),
        DeclareLaunchArgument('timeout', default_value='45.0'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_mocks', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('mocked_rover', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('geometry_profile', default_value='nav2_nominal'),
        DeclareLaunchArgument('legacy', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('clean', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])
