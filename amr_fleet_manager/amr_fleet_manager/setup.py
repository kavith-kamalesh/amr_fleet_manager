import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'amr_fleet_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'psutil'],
    zip_safe=True,
    maintainer='SIH Team',
    maintainer_email='user@example.com',
    description='Decentralized AMR Fleet Coordination',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'central_dispatcher = amr_fleet_manager.central_dispatcher:main',
            'mission_controller = amr_fleet_manager.mission_controller:main',
            'waypoint_nav = amr_fleet_manager.waypoint_nav_node:main',
            'spatial_mutex = amr_fleet_manager.spatial_mutex:main',
            'safety_fallback = amr_fleet_manager.safety_fallback:main',
            'orca_node = amr_fleet_manager.orca_node:main',
            'lidar_blindspot_fallback = amr_fleet_manager.lidar_blindspot_fallback:main',
        ],
    },
)
