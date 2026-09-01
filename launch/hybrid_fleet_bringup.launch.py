"""
Hybrid Edge-Cloud launch file.
Launches:
  - central_dispatcher.py in the root namespace (/central_fms logic,
    but the node itself runs unnamespaced since it's the global authority)
  - 3 fully isolated robot namespaces (/robot1, /robot2, /robot3), each
    with its own waypoint_nav_node, spatial_mutex, and safety_fallback
  - spawns each TurtleBot into Gazebo sequentially with a unique
    -robot_namespace argument to guarantee zero topic cross-talk
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    # Central FMS (root namespace, no robot-specific prefix)
    ld.add_action(Node(
        package='amr_fleet_manager',
        executable='central_dispatcher',
        name='central_dispatcher',
        output='screen',
    ))

    robot_configs = [
        {'name': 'robot1', 'id': 1, 'priority': 0.9, 'x': 0.0, 'y': 0.0},
        {'name': 'robot2', 'id': 2, 'priority': 0.6, 'x': 4.0, 'y': 0.0},
        {'name': 'robot3', 'id': 3, 'priority': 0.3, 'x': 0.0, 'y': 4.0},
    ]

    for i, cfg in enumerate(robot_configs):
        ns = cfg['name']

        # Sequential spawn with a small stagger delay per robot to avoid
        # Gazebo spawn collisions/race conditions
        spawn_entity = TimerAction(
            period=float(i * 3.0),
            actions=[Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name=f'spawn_{ns}',
                namespace=ns,
                arguments=[
                    '-entity', ns,
                    '-robot_namespace', ns,
                    '-x', str(cfg['x']), '-y', str(cfg['y']), '-z', '0.05',
                    '-topic', 'robot_description',
                ],
                output='screen',
            )]
        )
        ld.add_action(spawn_entity)

        ld.add_action(Node(
            package='amr_fleet_manager',
            executable='waypoint_nav_node',
            name='waypoint_nav_node',
            namespace=ns,
            parameters=[{'robot_id': cfg['id']}],
            output='screen',
        ))

        ld.add_action(Node(
            package='amr_fleet_manager',
            executable='spatial_mutex',
            name='spatial_mutex',
            namespace=ns,
            parameters=[{'robot_id': cfg['id'], 'priority': cfg['priority']}],
            output='screen',
        ))

        ld.add_action(Node(
            package='amr_fleet_manager',
            executable='safety_fallback',
            name='safety_fallback_watchdog',
            namespace=ns,
            output='screen',
        ))

    return ld
