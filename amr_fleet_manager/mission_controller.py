import math
import time
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


ROBOT_IDS = ["robot1", "robot2", "robot3"]

DISTANCE_THRESHOLD = 0.2
HEARTBEAT_TIMEOUT_SEC = 5.0
LINEAR_GAIN = 0.5
ANGULAR_GAIN = 1.5
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 1.0

STATE_IDLE = "IDLE"
STATE_WORKING = "WORKING"
STATE_OFFLINE = "OFFLINE"
STATE_CHARGING = "CHARGING"
STATE_SHIFT_CHANGE = "SHIFT_CHANGE"

# States in which this robot must be excluded from both movement
# and heartbeat-timeout policing (planned pause, not a failure).
PARKED_STATES = (STATE_CHARGING, STATE_SHIFT_CHANGE)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotWorker:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.state = STATE_IDLE

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.has_odom = False
        self.last_heartbeat = None

        self.current_task = None


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

        self.task_queue = [
            {'task_id': 'T1', 'x': 4.0, 'y': 2.0},
            {'task_id': 'T2', 'x': -3.0, 'y': 4.0},
            {'task_id': 'T3', 'x': 2.0, 'y': -3.0},
            {'task_id': 'T4', 'x': -4.0, 'y': -2.0},
            {'task_id': 'T5', 'x': 3.0, 'y': 3.0},
            {'task_id': 'T6', 'x': -2.0, 'y': -4.0},
        ]
        self.MAX_TASK_QUEUE_SIZE = 500

        self.workers = {rid: RobotWorker(rid) for rid in ROBOT_IDS}

        self.cmd_pubs = {}
        self.status_summary_pubs = {}
        for rid in ROBOT_IDS:
            self.cmd_pubs[rid] = self.create_publisher(Twist, f'/{rid}/cmd_vel', qos)
            self.status_summary_pubs[rid] = self.create_publisher(
                String, f'/{rid}/mission_status_summary', qos
            )
            self.create_subscription(
                Odometry, f'/{rid}/odom',
                lambda msg, r=rid: self.odom_callback(msg, r),
                qos
            )
            # Robot-owned self-report channel: CHARGING / SHIFT_CHANGE /
            # IDLE / OFFLINE, published by the robot's own
            # battery/shift-management logic (external to this node).
            self.create_subscription(
                String, f'/{rid}/health_status',
                lambda msg, r=rid: self.health_status_callback(msg, r),
                qos
            )

        # Fleet-wide, low-frequency broadcast of who is currently parked,
        # so edge nodes (spatial_mutex) can cheaply exclude them from
        # active conflict negotiation without subscribing to every
        # robot's health_status individually.
        self.charging_list_pub = self.create_publisher(String, '/fleet/charging_robots', qos)

        self.control_timer = self.create_timer(0.1, self.control_loop)
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

        if worker.state == STATE_OFFLINE:
            worker.state = STATE_IDLE
            worker.current_task = None
            self.get_logger().info(f"[{robot_id}] odom resumed -> back to IDLE")

    # ---------------- Robot-reported health/state channel ----------------

    def health_status_callback(self, msg: String, robot_id: str):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"[{robot_id}] malformed health_status payload, dropping")
            return

        status = data.get('status')
        reason = data.get('reason', '')

        if status in (STATE_CHARGING, STATE_SHIFT_CHANGE):
            self.pause_for_charge(robot_id, status, reason)
        elif status == STATE_IDLE:
            self.resume_from_charge(robot_id)
        elif status == STATE_OFFLINE:
            # Allow a robot to self-report a hard failure immediately,
            # rather than waiting the full heartbeat timeout.
            self.mark_offline(robot_id, reason=reason or "self_reported")
        # Any other/unknown status value is ignored, not treated as an error.

    def pause_for_charge(self, robot_id: str, status: str, reason: str = ""):
        worker = self.workers[robot_id]

        if worker.state in PARKED_STATES:
            return  # already paused, avoid double re-queueing the task

        self.get_logger().info(f"[{robot_id}] pausing for {status} (reason='{reason}')")
        worker.state = status

        # Stop it immediately and cleanly -- this is a planned pause,
        # not a failure, so we still explicitly zero its cmd_vel.
        self.cmd_pubs[robot_id].publish(Twist())

        if worker.current_task is not None:
            self.get_logger().info(
                f"[{robot_id}] reallocating task {worker.current_task['task_id']} "
                f"back to queue (graceful pause, not a failure)"
            )
            self.task_queue.insert(0, worker.current_task)
            worker.current_task = None

    def resume_from_charge(self, robot_id: str):
        worker = self.workers[robot_id]

        if worker.state not in PARKED_STATES:
            return  # nothing to resume from -- ignore stray IDLE reports

        self.get_logger().info(f"[{robot_id}] resumed from {worker.state} -> IDLE")
        worker.state = STATE_IDLE
        # Reset the heartbeat clock fresh: odom may not have been
        # flowing at all while parked, so don't let a stale timestamp
        # immediately trip the watchdog on the next tick.
        worker.last_heartbeat = time.time()

    # ---------------- Watchdog: failure detection + recovery ----------------

    def watchdog_loop(self):
        now = time.time()
        charging_ids = []

        for robot_id, worker in self.workers.items():
            if worker.state == STATE_OFFLINE:
                continue

            if worker.state in PARKED_STATES:
                # Planned pause: exempt from heartbeat policing entirely.
                # A charging robot deliberately stops publishing odom to
                # save bandwidth -- that silence must never be
                # misread as a failure.
                charging_ids.append(robot_id)
                continue

            if worker.last_heartbeat is None:
                continue

            if (now - worker.last_heartbeat) > HEARTBEAT_TIMEOUT_SEC:
                self.mark_offline(robot_id, reason="heartbeat_timeout")

        payload = json.dumps({"charging_robots": charging_ids, "timestamp": now})
        self.charging_list_pub.publish(String(data=payload))

    def mark_offline(self, robot_id: str, reason: str = "heartbeat_timeout"):
        worker = self.workers[robot_id]

        self.get_logger().warn(f"[{robot_id}] going OFFLINE (reason: {reason})")
        worker.state = STATE_OFFLINE
        self.cmd_pubs[robot_id].publish(Twist())

        if worker.current_task is not None:
            self.get_logger().warn(
                f"[{robot_id}] re-queuing task {worker.current_task['task_id']} "
                f"at front of task_queue"
            )
            self.task_queue.insert(0, worker.current_task)
            worker.current_task = None

    # ---------------- Task assignment ----------------

    def enqueue_task(self, task: dict):
        if len(self.task_queue) >= self.MAX_TASK_QUEUE_SIZE:
            self.get_logger().error(
                f"task_queue at MAX_TASK_QUEUE_SIZE ({self.MAX_TASK_QUEUE_SIZE}); "
                f"dropping task {task.get('task_id')}"
            )
            return
        self.task_queue.append(task)

    def assign_task_if_idle(self, robot_id: str):
        worker = self.workers[robot_id]

        if worker.state != STATE_IDLE:
            return
        if not self.task_queue:
            return

        task = self.task_queue.pop(0)
        worker.current_task = task
        worker.state = STATE_WORKING

        self.get_logger().info(
            f"[{robot_id}] assigned task {task['task_id']} -> ({task['x']}, {task['y']})"
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
        linear_speed = LINEAR_GAIN * distance
        linear_speed *= max(0.0, math.cos(angle_error))
        cmd.linear.x = max(-MAX_LINEAR_SPEED, min(MAX_LINEAR_SPEED, linear_speed))

        angular_speed = ANGULAR_GAIN * angle_error
        cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angular_speed))

        self.cmd_pubs[robot_id].publish(cmd)

    def complete_task(self, robot_id: str):
        worker = self.workers[robot_id]
        self.cmd_pubs[robot_id].publish(Twist())

        if worker.current_task is not None:
            self.get_logger().info(f"[{robot_id}] completed task {worker.current_task['task_id']}")
        worker.current_task = None
        worker.state = STATE_IDLE

    # ---------------- Main control loop ----------------

    def control_loop(self):
        for robot_id, worker in self.workers.items():
            if worker.state == STATE_OFFLINE:
                continue
            if worker.state in PARKED_STATES:
                continue  # no assignment, no movement while parked

            if worker.state == STATE_IDLE:
                self.assign_task_if_idle(robot_id)
                continue

            if worker.state == STATE_WORKING:
                if not worker.has_odom:
                    continue
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
