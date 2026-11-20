from glob import glob
from setuptools import setup

setup(
    name='pathplanner_mock_tools', version='0.1.0', packages=['pathplanner_mock_tools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/pathplanner_mock_tools']),
        ('share/pathplanner_mock_tools', ['package.xml']),
        ('share/pathplanner_mock_tools/maps', glob('maps/*')),
        ('share/pathplanner_mock_tools/custom_maps', glob('custom_maps/*')),
        ('share/pathplanner_mock_tools/launch', glob('launch/*.launch.py')),
        ('share/pathplanner_mock_tools/rviz', glob('rviz/*.rviz')),
        ('share/pathplanner_mock_tools', ['README.md']),
    ],
    install_requires=['setuptools'],
    maintainer='Rover team', maintainer_email='team@example.com',
    description='Test-only pathplanning dependency mocks and fixtures',
    license='Apache-2.0',
    entry_points={'console_scripts': ['mock_rover_sim = pathplanner_mock_tools.mock_rover_sim:main', 'mock_perception = pathplanner_mock_tools.mock_perception:main', 'drop_stone = pathplanner_mock_tools.drop_stone:main', 'algo_node = pathplanner_mock_tools.algo_node:main', 'mock_base = pathplanner_mock_tools.mock_base:main', 'generate_benchmark_maps = pathplanner_mock_tools.generate_benchmark_maps:main']},
)
