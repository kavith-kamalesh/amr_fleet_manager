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
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools', 'psutil', 'onnxruntime'],
    zip_safe=True,
    maintainer='SIH Team',
    maintainer_email='user@example.com',
    description='Decentralized AMR Fleet Coordination via ROS 2 DDS',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_kinematic_robot = amr_fleet_manager.sim_kinematic_robot:main',
            'central_dispatcher = amr_fleet_manager.central_dispatcher:main',
            'mission_controller = amr_fleet_manager.mission_controller:main',
            'waypoint_nav_node = amr_fleet_manager.waypoint_nav_node:main',
            'stop_and_wait_node = amr_fleet_manager.stop_and_wait_node:main',
            'battery_monitor = amr_fleet_manager.battery_monitor:main',
            'health_monitor = amr_fleet_manager.health_monitor:main',
            'alert_dispatcher = amr_fleet_manager.alert_dispatcher:main',
            'task_auctioneer = amr_fleet_manager.task_auctioneer:main',
            'local_bidder = amr_fleet_manager.local_bidder:main',
            'dynamic_obstacle_layer = amr_fleet_manager.dynamic_obstacle_layer:main',
            'spatial_mutex = amr_fleet_manager.spatial_mutex:main',
            'safety_fallback = amr_fleet_manager.safety_fallback:main',
            'orca_node = amr_fleet_manager.orca_node:main',
            'lidar_blindspot_fallback = amr_fleet_manager.lidar_blindspot_fallback:main',
        ],
    },
)
