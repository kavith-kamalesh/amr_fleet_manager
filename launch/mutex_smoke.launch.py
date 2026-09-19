from launch import LaunchDescription
from launch_ros.actions import Node

ROBOTS = [
    {'name': 'robot1', 'id': 1, 'priority': 0.9, 'x': 0.0, 'y': 0.0},
    {'name': 'robot2', 'id': 2, 'priority': 0.6, 'x': 4.0, 'y': 0.0},
    {'name': 'robot3', 'id': 3, 'priority': 0.3, 'x': 0.0, 'y': 4.0},
]


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(Node(package='amr_fleet_manager', executable='central_dispatcher',
                       name='central_dispatcher', output='screen'))
    for c in ROBOTS:
        ns = c['name']
        ld.add_action(Node(package='amr_fleet_manager', executable='sim_kinematic_robot',
                           name='sim_kinematic_robot', namespace=ns, output='screen'))
        ld.add_action(Node(package='amr_fleet_manager', executable='waypoint_nav_node',
                           name='waypoint_nav_node', namespace=ns, output='screen',
                           parameters=[{'robot_id': c['id'],
                                        'spawn_offset_x': c['x'], 'spawn_offset_y': c['y']}]))
        ld.add_action(Node(package='amr_fleet_manager', executable='spatial_mutex',
                           name='spatial_mutex', namespace=ns, output='screen',
                           parameters=[{'robot_id': c['id'], 'priority': c['priority']}]))
        ld.add_action(Node(package='amr_fleet_manager', executable='safety_fallback',
                           name='safety_supervisor', namespace=ns, output='screen',
                           parameters=[{'require_scan': False}]))
    return ld
