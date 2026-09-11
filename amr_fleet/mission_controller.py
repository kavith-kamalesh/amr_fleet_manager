"""
Mission Controller with Dynamic Task Reallocation (Self-Healing Fleet).

Preserves the existing architecture:
  - Namespace isolation: /robot1, /robot2, /robot3
  - Inputs:  /{robot}/odom      (nav_msgs/msg/Odometry)
  - Outputs: /{robot}/cmd_vel   (geometry_msgs/msg/Twist)
  - Proportional steering navigation (0.2m distance threshold, angle normalization)

New in this version:
  - No hardcoded per-robot targets. A single global task_queue feeds
    waypoints to whichever robot is IDLE.
  - Per-robot state machine: IDLE / WORKING / OFFLINE.
  - Heartbeat watchdog: if a robot's /odom goes silent for > 5.0s, it is
    marked OFFLINE, its cmd_vel is halted, and its in-progress task is
    re-inserted at the front of the queue for another robot to pick up.
  - If an OFFLINE robot's odom resumes, it returns to IDLE and can be
    assigned new work.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


ROBOT_IDS = ["robot1", "robot2", "robot3"]

DISTANCE_THRESHOLD = 0.2        # meters -- close enough to call a waypoint reached
HEARTBEAT_TIMEOUT_SEC = 5.0     # seconds -- no odom in this window -> OFFLINE
LINEAR_GAIN = 0.5
ANGULAR_GAIN = 1.5
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 1.0

STATE_IDLE = "IDLE"
STATE_WORKING = "WORKING"
STATE_OFFLINE = "OFFLINE"


def normalize_angle(angle):
    """Normalize an angle to the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    """Extract yaw (rotation about Z) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotWorker:
    """Per-robot runtime state: position, task assignment, health."""

    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.state = STATE_IDLE

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.has_odom = False
        self.last_heartbeat = None  # time.time() of last odom message

        self.current_task = None  # dict: {'task_id', 'x', 'y'}


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

        # ---------------- Dynamic global task queue ----------------
        # No per-robot static targets. Tasks are pulled from the FRONT
        # of this list and handed to whichever robot is IDLE.
        self.task_queue = [
            {'task_id': 'T1', 'x': 4.0, 'y': 2.0},
            {'task_id': 'T2', 'x': -3.0, 'y': 4.0},
            {'task_id': 'T3', 'x': 2.0, 'y': -3.0},
            {'task_id': 'T4', 'x': -4.0, 'y': -2.0},
            {'task_id': 'T5', 'x': 3.0, 'y': 3.0},
            {'task_id': 'T6', 'x': -2.0, 'y': -4.0},
        ]

        self.workers = {rid: RobotWorker(rid) for rid in ROBOT_IDS}

        self.cmd_pubs = {}
        for rid in ROBOT_IDS:
            self.cmd_pubs[rid] = self.create_publisher(Twist, f'/{rid}/cmd_vel', qos)
            self.create_subscription(
                Odometry, f'/{rid}/odom',
                lambda msg, r=rid: self.odom_callback(msg, r),
                qos
            )

        # Control loop: navigation + task assignment
        self.control_timer = self.create_timer(0.1, self.control_loop)
        # Watchdog loop: heartbeat/failure detection, runs independently
        # of the control loop so a stalled navigation calc never delays
        # failure detection.
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_loop)

        self.get_logger().info(
            f"MissionController up. Robots: {ROBOT_IDS} | "
            f"Initial task_queue size: {len(self.task_queue)}"
        )

    # ---------------- Odometry / heartbeat ----------------

    def odom_callback(self, msg: Odometry, robot_id: str):
        worker = self.workers[robot_id]

        worker.x = msg.pose.pose.position.x
        worker.y = msg.pose.pose.position.y
        worker.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        worker.has_odom = True
        worker.last_heartbeat = time.time()

        # If this robot was OFFLINE and odom has resumed, bring it back.
        # It becomes IDLE (not WORKING) even if it still had a task
        # recorded locally -- that task was already reassigned to the
        # queue front by the watchdog, so this robot starts fresh.
        if worker.state == STATE_OFFLINE:
            worker.state = STATE_IDLE
            worker.current_task = None
            self.get_logger().info(f"[{robot_id}] odom resumed -> back to IDLE")

    # ---------------- Watchdog: failure detection + recovery ----------------

    def watchdog_loop(self):
        now = time.time()

        for robot_id, worker in self.workers.items():
            if worker.state == STATE_OFFLINE:
                continue  # already handled, waiting for odom to resume

            if worker.last_heartbeat is None:
                continue  # never received odom yet, nothing to time out

            elapsed = now - worker.last_heartbeat
            if elapsed > HEARTBEAT_TIMEOUT_SEC:
                self.mark_offline(robot_id)

    def mark_offline(self, robot_id: str):
        worker = self.workers[robot_id]

        self.get_logger().warn(
            f"[{robot_id}] heartbeat timeout (> {HEARTBEAT_TIMEOUT_SEC}s) -> marking OFFLINE"
        )
        worker.state = STATE_OFFLINE

        # Halt publishing to this robot's cmd_vel immediately.
        self.cmd_pubs[robot_id].publish(Twist())

        # If it had an active task, put it back at the FRONT of the
        # queue so the next available robot picks it up first.
        if worker.current_task is not None:
            self.get_logger().warn(
                f"[{robot_id}] re-queuing task {worker.current_task['task_id']} "
                f"at front of task_queue"
            )
            self.task_queue.insert(0, worker.current_task)
            worker.current_task = None

    # ---------------- Task assignment ----------------

    def assign_task_if_idle(self, robot_id: str):
        worker = self.workers[robot_id]

        if worker.state != STATE_IDLE:
            return
        if not self.task_queue:
            return  # nothing left to assign

        task = self.task_queue.pop(0)
        worker.current_task = task
        worker.state = STATE_WORKING

        self.get_logger().info(
            f"[{robot_id}] assigned task {task['task_id']} -> "
            f"({task['x']}, {task['y']})"
        )

    # ---------------- Navigation: proportional steering ----------------

    def navigate_to_task(self, robot_id: str):
        worker = self.workers[robot_id]
        task = worker.current_task

        dx = task['x'] - worker.x
        dy = task['y'] - worker.y
        distance = math.hypot(dx, dy)

        if distance <= DISTANCE_THRESHOLD:
            self.complete_task(robot_id)
            return

        target_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(target_angle - worker.yaw)

        cmd = Twist()
        # Slow down linear speed as heading error grows, so the robot
        # turns toward the target before committing to forward motion.
        linear_speed = LINEAR_GAIN * distance
        linear_speed *= max(0.0, math.cos(angle_error))
        cmd.linear.x = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, linear_speed))

        angular_speed = ANGULAR_GAIN * angle_error
        cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angular_speed))

        self.cmd_pubs[robot_id].publish(cmd)

    def complete_task(self, robot_id: str):
        worker = self.workers[robot_id]

        self.cmd_pubs[robot_id].publish(Twist())  # stop

        if worker.current_task is not None:
            self.get_logger().info(
                f"[{robot_id}] completed task {worker.current_task['task_id']}"
            )
        worker.current_task = None
        worker.state = STATE_IDLE

    # ---------------- Main control loop ----------------

    def control_loop(self):
        for robot_id, worker in self.workers.items():
            if worker.state == STATE_OFFLINE:
                continue  # never publish motion commands to an offline robot

            if worker.state == STATE_IDLE:
                self.assign_task_if_idle(robot_id)
                continue  # if just assigned, navigate starts next tick

            if worker.state == STATE_WORKING:
                if not worker.has_odom:
                    continue  # no position fix yet, wait
                self.navigate_to_task(robot_id)


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
