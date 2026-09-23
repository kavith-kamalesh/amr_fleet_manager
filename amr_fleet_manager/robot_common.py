# ---------------- Canonical robot identity ----------------

ROBOT_IDS = ["robot1", "robot2", "robot3"]


def robot_topic(robot_id: str, topic: str) -> str:
    
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
    

    def __init__(self, robot_ids=None):
        self.robots = {rid: RobotState(rid) for rid in (robot_ids or ROBOT_IDS)}

    def get(self, robot_id: str) -> RobotState:
        return self.robots.get(robot_id)

    def to_dict(self):
        return {rid: state.to_dict() for rid, state in self.robots.items()}


class Task:
    

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

MUTEX_CLEAR = "CLEAR"
MUTEX_WAIT = "WAIT"
MUTEX_REROUTE = "REROUTE_REQUESTED"
MUTEX_PARKED = "PARKED"

# ---------------- Intent message signing (HMAC-SHA256) ----------------
# Pre-shared-key MVP: every robot is launched with the same hmac_key
# parameter. This stops the spoofed-intent attack demonstrated by
# verify_log_not_process.py (an unsigned fake peer at priority 1.0
# could previously freeze the whole fleet). Roadmap: SROS2 / DDS-Security
# with per-robot X.509 identities and real key rotation -- a PSK is an
# honest MVP, not a claim of full PKI.

import hmac
import hashlib
import json


def _canonicalize(payload: dict) -> bytes:
    return json.dumps(payload, sort_keys=True, separators=(',', ':')).encode('utf-8')


def sign_payload(payload: dict, key: str) -> str:
    return hmac.new(key.encode('utf-8'), _canonicalize(payload), hashlib.sha256).hexdigest()


def verify_payload(payload: dict, signature: str, key: str) -> bool:
    if not signature:
        return False
    return hmac.compare_digest(sign_payload(payload, key), signature)
