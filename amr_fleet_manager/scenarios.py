"""Goal tables for the smoke test (pure data, so tests can import it without rclpy).

Goals are world coordinates. 'swap' is the original head-on scenario (robot1 and robot2 exchange
places). It is a hard, symmetric case: reactive replanning fails liveness in some layouts of it,
in the benchmark (parked-robot model) and in the ROS runs. 'crossing' is perpendicular traffic where
every goal is a distinct bay off the others' paths. Report both.
"""

SWAP = {
    'robot1': (3.0, 0.0),
    'robot2': (1.0, 0.0),
    'robot3': (2.0, 2.0),
}

CROSSING = {
    'robot1': (6.0, 0.0),
    'robot2': (4.0, 6.0),
    'robot3': (8.0, 4.0),
}

SCENARIOS = {'swap': SWAP, 'crossing': CROSSING}
SPAWN = {'robot1': (0.0, 0.0), 'robot2': (4.0, 0.0), 'robot3': (0.0, 4.0)}
