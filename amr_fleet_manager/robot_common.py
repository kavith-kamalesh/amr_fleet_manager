"""
Shared constants, naming conventions, and data structures for the AMR
fleet. Import from this module instead of hardcoding robot IDs, topic
names, or frame names anywhere else in the package.

This is the single source of truth for the hardware abstraction layer:
the algorithm should only ever depend on these names, never on whether
the underlying robot is a Gazebo simulation or a real Raspberry Pi AMR.
"""

# ---------------- Canonical robot identity ----------------

ROBOT_IDS = ["robot1", "robot2", "robot3"]


def robot_topic(robot_id: str, topic: str) -> str:
    """
    Build a namespaced topic name for a given robot.
    robot_topic("robot1", "scan") -> "/robot1/scan"
    Note: in ROS2, namespacing via launch already achieves this
    automatically for nodes running inside that namespace -- this
    helper is for cases where a node needs to explicitly address a
    PEER robot's topic (e.g. cross-robot introspection/tools), not for
    a robot's own topics.
    """
    return f"/{robot_id}/{topic}"


# ---------------- Frames ----------------

ODOM_FRAME = "odom"
BASE_FRAME = "base_footprint"
MAP_FRAME = "map"

# ---------------- Motion limits ----------------

MAX_LINEAR_VELOCITY = 1.0   # m/s
MAX_ANGULAR_VELOCITY = 1.5  # rad/s

# ---------------- Coordination thresholds ----------------

SAFE_DISTANCE = 0.75        # meters -- comfortable inter-robot spacing
COLLISION_THRESHOLD = 0.50  # meters -- hard danger zone

# ---------------- Hardware abstraction names ----------------
# These describe the LOGICAL role, not the physical implementation.
# In Gazebo, each maps to a simulated topic/plugin; on real hardware,
# each maps to physical sensors/actuators on the Raspberry Pi. The
# algorithm code should never need to know which one it's talking to.

EDGE_COMPUTER = "edge_computer"        # sim: ROS2 node on dev machine | real: Raspberry Pi
LIDAR_INTERFACE = "lidar"              # sim: Gazebo LiDAR plugin      | real: physical 2D LiDAR
IMU_INTERFACE = "imu"                  # sim: Gazebo IMU plugin        | real: physical IMU
MOTOR_INTERFACE = "cmd_vel"            # sim: Gazebo diff-drive plugin | real: motor driver
ENCODER_INTERFACE = "odom"             # sim: Gazebo joint states      | real: wheel encoders
BATTERY_INTERFACE = "battery"          # sim: simulated constant       | real: battery/BMS
FLEET_INTERFACE = "fleet_comms"        # sim: ROS2 topics              | real: ESP-NOW / ROS2


# ---------------- Structured robot/fleet state ----------------

class RobotState:
    """
    Canonical per-robot state object. Nodes that need to reason about
    a robot's full state (position, velocity, task, health) should
    build/consume this instead of passing scattered loose variables.
    """

    def __init__(self, robot_id: str):
        self.robot_id = robot_id

        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation = 0.0  # theta, radians

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.lidar_ranges = []

        self.imu_linear_acceleration = None
        self.imu_angular_velocity = None
        self.imu_orientation = None

        self.battery_level = 1.0  # 0.0-1.0, simulated as full unless updated

        self.current_task = None
        self.robot_status = "IDLE"  # IDLE | MOVING | WAITING | REROUTING | ARRIVED

    @property
    def pose(self):
        return {"x": self.position_x, "y": self.position_y, "theta": self.orientation}

    def to_dict(self):
        return {
            "robot_id": self.robot_id,
            "position": {"x": self.position_x, "y": self.position_y},
            "orientation": self.orientation,
            "linear_velocity": self.linear_velocity,
            "angular_velocity": self.angular_velocity,
            "battery_level": self.battery_level,
            "current_task": self.current_task,
            "status": self.robot_status,
        }


class FleetState:
    """
    Canonical fleet-wide state container. Central FMS or any
    cross-robot logic should read/write through this rather than
    maintaining separate ad-hoc dicts.
    """

    def __init__(self, robot_ids=None):
        self.robots = {rid: RobotState(rid) for rid in (robot_ids or ROBOT_IDS)}

    def get(self, robot_id: str) -> RobotState:
        return self.robots.get(robot_id)

    def to_dict(self):
        return {rid: state.to_dict() for rid, state in self.robots.items()}


class Task:
    """Canonical task object for fleet-level task allocation."""

    def __init__(self, task_id, task_type, location, priority=1, assigned_robot=None):
        self.task_id = task_id
        self.task_type = task_type
        self.location = location  # (x, y)
        self.priority = priority
        self.assigned_robot = assigned_robot
        self.status = "PENDING"  # PENDING | ASSIGNED | IN_PROGRESS | DONE

    def to_dict(self):
        return {
            "task_id": self.task_id,
            "task_type": self.task_type,
            "location": self.location,
            "priority": self.priority,
            "assigned_robot": self.assigned_robot,
            "status": self.status,
        }
