from glob import glob
from setuptools import setup

setup(
    name='slam_mock_tools', version='0.1.0', packages=['slam_mock_tools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/slam_mock_tools']),
        ('share/slam_mock_tools', ['package.xml']),
    ],
    install_requires=['setuptools'],
    maintainer='Rover team', maintainer_email='team@example.com',
    description='Test-only slam dependency mocks and fixtures',
    license='MIT',
    entry_points={'console_scripts': ['mock_aruco_publisher = slam_mock_tools.mock_aruco_publisher:main', 'costmap_test_stub = slam_mock_tools.costmap_test_stub:main']},
)
