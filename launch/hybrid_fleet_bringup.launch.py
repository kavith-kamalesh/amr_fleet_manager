"""
Hybrid Edge-Cloud launch file.
Launches:
  - mission_controller.py in the root namespace: central task allocation
    and fleet-health watchdog ONLY. It assigns goal_pose to idle robots
    and re-queues work on failure/charge-pause -- it does NOT compute
    cmd_vel for any robot, so this is a legitimate WMS-style central
    authority, not a violation of the "no central server" traffic
    coordination requirement. (central_dispatcher.py is superseded by
    this node and is no longer launched -- it remains in the repo only
    as the original scripted-demo reference.)
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

# Pre-shared HMAC key for spatial_mutex intent signing. Every robot in
# the fleet must be launched with the SAME key, or they'll reject each
# other's signed intents as invalid. Roadmap: SROS2 / per-robot X.509.
FLEET_SHARED_KEY = 'sih26123-team-codecircuit-demo-key'


def generate_launch_description():
    ld = LaunchDescription()

    # Central FMS (root namespace, no robot-specific prefix).
    # Task allocation + watchdog only -- no motion control. See module
    # docstring above for why this is architecturally fine.
    ld.add_action(Node(
        package='amr_fleet_manager',
        executable='mission_controller',
        name='mission_controller',
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
            output='screen',
        ))

        ld.add_action(Node(
            package='amr_fleet_manager',
            executable='battery_monitor',
            name='battery_monitor',
            namespace=ns,
            parameters=[{'robot_id': cfg['id']}],
            output='screen',
        ))

        ld.add_action(Node(
            package='amr_fleet_manager',
            executable='health_monitor',
            name='health_monitor',
            namespace=ns,
            parameters=[{'robot_id': cfg['id']}],
            output='screen',
        ))

        ld.add_action(Node(
            package='amr_fleet_manager',
            executable='dynamic_obstacle_layer',
            name='dynamic_obstacle_layer',
            namespace=ns,
            parameters=[{'robot_id': cfg['id']}],
            output='screen',
        ))

        ld.add_action(Node(
            package='amr_fleet_manager',
            executable='spatial_mutex',
            name='spatial_mutex',
            namespace=ns,
            parameters=[{
                'robot_id': cfg['id'],
                'priority': cfg['priority'],
                'hmac_key': FLEET_SHARED_KEY,
                'priority_aging_rate': 0.05,
            }],
            output='screen',
        ))

        ld.add_action(Node(
            package='amr_fleet_manager',
            executable='safety_fallback',
            name='safety_fallback_watchdog',
            namespace=ns,
            output='screen',
        ))

    ld.add_action(Node(
        package='amr_fleet_manager',
        executable='alert_dispatcher',
        name='alert_dispatcher',
        output='screen',
    ))

    return ld
