"""
Decentralised PIBT fleet bring-up (replaces spatial_mutex for traffic control).

Per robot namespace /robotN:
  pibt_fleet_node   negotiates one grid move per tick with the other robots
  waypoint_nav_node drives to the waypoint (its `goal_pose` is REMAPPED to
                    `nav_waypoint`, so the dispatcher's goals reach pibt_fleet_node)
  safety_fallback   LiDAR watchdog -- keep this running underneath

Global: central_dispatcher (hands out TASK GOALS only; coordination is local).

Usage (Gazebo clock):  ros2 launch amr_fleet_manager pibt_fleet_bringup.launch.py
Without spawning:      ... spawn:=false   (if your robots are already in Gazebo)
"""
import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ROBOTS = [
    {'name': 'robot1', 'id': 1, 'priority': 0.9, 'x': 0.0, 'y': 0.0},
    {'name': 'robot2', 'id': 2, 'priority': 0.6, 'x': 4.0, 'y': 0.0},
    {'name': 'robot3', 'id': 3, 'priority': 0.3, 'x': 0.0, 'y': 4.0},
]
CELL_SIZE = 1.0     # metres per grid cell; spawn positions must sit on cell centres


def generate_launch_description():
    # The whole fleet, known to everyone at start-up (see fleet_protocol.seed_peer)
    roster = {str(r['id']): {'cell': [int(round(r['x'] / CELL_SIZE)),
                                      int(round(r['y'] / CELL_SIZE))],
                             'prio': r['priority']} for r in ROBOTS}
    roster_json = json.dumps(roster)

    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('spawn', default_value='true'),
        DeclareLaunchArgument('tick_period', default_value='8.0'),
        DeclareLaunchArgument('grid_w', default_value='6'),
        DeclareLaunchArgument('grid_h', default_value='6'),
        Node(package='amr_fleet_manager', executable='central_dispatcher',
             name='central_dispatcher', output='screen',
             parameters=[{'use_sim_time': sim_time}]),
    ])

    for i, r in enumerate(ROBOTS):
        ns = r['name']

        ld.add_action(TimerAction(
            period=float(i * 3.0),
            condition=IfCondition(LaunchConfiguration('spawn')),
            actions=[Node(
                package='gazebo_ros', executable='spawn_entity.py',
                name=f'spawn_{ns}', namespace=ns,
                arguments=['-entity', ns, '-robot_namespace', ns,
                           '-x', str(r['x']), '-y', str(r['y']), '-z', '0.05',
                           '-topic', 'robot_description'],
                output='screen')]))

        ld.add_action(Node(
            package='amr_fleet_manager', executable='waypoint_nav_node',
            name='waypoint_nav_node', namespace=ns,
            remappings=[('goal_pose', 'nav_waypoint')],
            parameters=[{'spawn_offset_x': r['x'], 'spawn_offset_y': r['y'],
                         'use_sim_time': sim_time}],
            output='screen'))

        ld.add_action(Node(
            package='amr_fleet_manager', executable='pibt_fleet_node',
            name='pibt_fleet_node', namespace=ns,
            parameters=[{
                'robot_id': r['id'], 'priority': r['priority'],
                'spawn_offset_x': r['x'], 'spawn_offset_y': r['y'],
                'cell_size': CELL_SIZE, 'origin_x': 0.0, 'origin_y': 0.0,
                'grid_w': ParameterValue(LaunchConfiguration('grid_w'), value_type=int),
                'grid_h': ParameterValue(LaunchConfiguration('grid_h'), value_type=int),
                'roster_json': roster_json,
                'tick_period': ParameterValue(LaunchConfiguration('tick_period'),
                                              value_type=float),
                'use_sim_time': sim_time}],
            output='screen'))

        ld.add_action(Node(
            package='amr_fleet_manager', executable='safety_fallback',
            name='safety_fallback_watchdog', namespace=ns,
            parameters=[{'use_sim_time': sim_time}], output='screen'))

    return ld
