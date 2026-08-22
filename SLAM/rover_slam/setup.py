import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_slam'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Saif',
    maintainer_email='saif@erc-rover.org',
    description='ERC Mars Rover SLAM Subsystem Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_ticks_to_odom = rover_slam.encoder_ticks_to_odom:main',
            'heuristic_slip_checker = rover_slam.heuristic_slip_checker:main',
            'ekf_validator = rover_slam.ekf_validator:main',
            'costmap_test_stub = rover_slam.costmap_test_stub:main',
            'mock_aruco_publisher = rover_slam.mock_aruco_publisher:main',
        ],
    },

)
