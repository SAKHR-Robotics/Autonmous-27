from glob import glob
from setuptools import setup
setup(name='slam_test_tools',version='0.1.0',packages=['slam_test_tools'],
 data_files=[('share/ament_index/resource_index/packages',['resource/slam_test_tools']),
 ('share/slam_test_tools',['package.xml']),('share/slam_test_tools/launch',glob('launch/*.launch.py'))],
 install_requires=['setuptools'],license='MIT',maintainer='Rover team',maintainer_email='team@example.com',
 description='Explicit synthetic SLAM fixtures, not production nodes',
 entry_points={'console_scripts':['costmap_test_stub = slam_test_tools.costmap_test_stub:main',
 'mock_aruco_publisher = slam_test_tools.mock_aruco_publisher:main']})
