import os
from glob import glob
from setuptools import setup

package_name = 'amr_fleet'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kavith',
    maintainer_email='user@todo.todo',
    description='Edge AI Fleet Manager for AMR',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_controller = amr_fleet.mission_controller:main',
            'waypoint_nav = amr_fleet.waypoint_nav_node:main',
            'spatial_mutex = amr_fleet.spatial_mutex:main',
        ],
    },
)
