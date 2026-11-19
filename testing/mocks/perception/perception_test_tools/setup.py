from glob import glob
from setuptools import setup
setup(name='perception_test_tools',version='0.1.0',packages=['perception_test_tools'],
 data_files=[('share/ament_index/resource_index/packages',['resource/perception_test_tools']),
 ('share/perception_test_tools',['package.xml']),('share/perception_test_tools/launch',glob('launch/*.launch.py')),('share/perception_test_tools/rviz',glob('rviz/*.rviz'))],
 install_requires=['setuptools'],license='Apache-2.0',maintainer='Rover team',maintainer_email='team@example.com',
 description='Isolated synthetic perception sensor fixtures',
 entry_points={'console_scripts':['sensor_inputs = perception_test_tools.sensor_inputs:main']})
