from setuptools import setup

package_name = 'perception_benchmark_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    license='Apache-2.0',
    maintainer='Rover team',
    maintainer_email='team@example.com',
    description='Offline terrain helper timing benchmark; not node acceptance validation',
    entry_points={'console_scripts': [
        'terrain_benchmark = perception_benchmark_tools.terrain_benchmark:main',
    ]},
)
