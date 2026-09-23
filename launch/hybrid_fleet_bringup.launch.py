"""
Hybrid Edge-Cloud launch file -- Gazebo Harmonic (gz-sim) port.

Launches:
  - gz-sim itself, loaded with worlds/sih_warehouse.world. This was
    NEVER actually wired in before -- IncludeLaunchDescription and
    PythonLaunchDescriptionSource were imported in the original file but
    never used in the body, meaning this launch file could not have
    started a simulator on its own even before the Classic/Harmonic
    mismatch. Fixed here.
  - mission_controller.py in the root namespace: central task allocation
    and fleet-health watchdog ONLY. No motion control -- see the
    architecture note in the git history for why that's fine.
  - 3 fully isolated robot namespaces (/robot1, /robot2, /robot3), each
    with its own waypoint_nav_node, spatial_mutex, and safety_fallback,
    spawned from whatever 'robot_description' topic is being published
    per-namespace -- this file does not assume any specific robot model,
    same as the original design.
  - a ros_gz_bridge per robot for cmd_vel/odom/scan, plus one shared
    /clock bridge. Gazebo Classic auto-bridged these via gazebo_ros
    plugins; Harmonic requires them declared explicitly, which is most
    of what "porting to ros_gz_sim" actually means in practice.

WHAT'S VERIFIED-GENERIC vs WHAT NEEDS CHECKING AGAINST YOUR ROBOT:
  - The gz-sim launch, the 'create' spawn call, and the /clock bridge
    are correct for ANY robot -- they don't reference robot internals.
  - The cmd_vel/odom/scan bridge topics below
    (/model/<robot>/cmd_vel, /model/<robot>/odometry, /model/<robot>/scan)
    are the DEFAULT topic names gz-sim's diff-drive/odometry/sensor
    system plugins use when a robot's SDF/URDF does NOT override them
    with a custom <topic> tag. If your robot's description sets custom
    topic names, update BRIDGE_ARGS below to match -- this is the one
    piece that's genuinely robot-specific and I have no visibility into
    your actual robot description to verify against.
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

    # ---------------- Start Gazebo Harmonic itself ----------------
    # Requires worlds/sih_warehouse.world to be installed as package
    # share data -- add a worlds glob to setup.py's data_files if it
    # isn't already there:
    #   (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    world_path = os.path.join(
        get_package_share_directory('amr_fleet_manager'), 'worlds', 'sih_warehouse.world'
    )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )
    ld.add_action(gz_sim)

    # Shared clock bridge -- one per simulation, not per robot. Without
    # this, ROS 2 nodes never see /clock and anything using sim time
    # (which everything here does, indirectly, via wall-clock timestamps
    # in spatial_mutex's reservation windows) drifts from the simulator.
    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    ))

    # Central FMS (root namespace, no robot-specific prefix).
    # Task allocation + watchdog only -- no motion control.
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
        # spawn race conditions.
        spawn_entity = TimerAction(
            period=float(i * 3.0) + 2.0,  # +2.0: give gz-sim itself time to come up first
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_{ns}',
                namespace=ns,
                arguments=[
                    '-name', ns,
                    '-topic', 'robot_description',
                    '-x', str(cfg['x']), '-y', str(cfg['y']), '-z', '0.05',
                ],
                output='screen',
            )]
        )
        ld.add_action(spawn_entity)

        # gz-sim's default topic names for a robot named e.g. 'robot1'
        # spawned with -name robot1. Verify against your robot's SDF if
        # it overrides these with custom <topic> tags on its diff-drive /
        # odometry / sensor plugins -- see module docstring.
        BRIDGE_ARGS = [
            f'/model/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/model/{ns}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/model/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ]
        bridge = TimerAction(
            period=float(i * 3.0) + 3.0,  # after this robot's own spawn
            actions=[Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                namespace=ns,
                arguments=BRIDGE_ARGS,
                remappings=[
                    (f'/model/{ns}/cmd_vel', 'cmd_vel'),
                    (f'/model/{ns}/odometry', 'odom'),
                    (f'/model/{ns}/scan', 'scan'),
                ],
                output='screen',
            )]
        )
        ld.add_action(bridge)

        ld.add_action(Node(
            package='amr_fleet_manager',
            executable='waypoint_nav_node',
            name='waypoint_nav_node',
            namespace=ns,
            parameters=[{
                'robot_id': cfg['id'],
                'spawn_offset_x': cfg['x'],
                'spawn_offset_y': cfg['y'],
            }],
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

    return ld
