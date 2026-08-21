from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'global_path_benchmarking'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saif',
    maintainer_email='saif@todo.todo',
    description='A standalone black-box benchmarking tool for ROS 2 global path planning modules.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'benchmarking_node = global_path_benchmarking.benchmarking_node:main',
            'testing_node = global_path_benchmarking.testing_node:main',
        ],
    },
)
