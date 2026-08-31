from setuptools import find_packages, setup

package_name = 'amr_fleet_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SIH Team',
    maintainer_email='user@example.com',
    description='Decentralized AMR Fleet Coordination',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orca_node = amr_fleet_manager.orca_node:main',
            'safety_fallback = amr_fleet_manager.safety_fallback:main',
        ],
    },
)
