import time
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


ROBOT_IDS = ["robot1", "robot2", "robot3"]

HEARTBEAT_TIMEOUT_SEC = 5.0

STATE_IDLE = "IDLE"
STATE_WORKING = "WORKING"
STATE_OFFLINE = "OFFLINE"
STATE_CHARGING = "CHARGING"
STATE_SHIFT_CHANGE = "SHIFT_CHANGE"

PARKED_STATES = (STATE_CHARGING, STATE_SHIFT_CHANGE)


class RobotWorker:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.state = STATE_IDLE

        # Position is tracked for fleet-awareness / dashboard purposes only.
        # It is NOT used to compute steering -- that decision belongs
        # entirely to this robot's own waypoint_nav_node + spatial_mutex.
        self.x = 0.0
        self.y = 0.0

        self.has_odom = False
        self.last_heartbeat = None

        self.current_task = None


class MissionController(Node):
    """
    Central task allocator and fleet-health watchdog -- the WMS-facing
    layer. It decides WHICH robot does WHICH task and re-queues work when
    a robot goes offline or pauses to charge. It does NOT decide HOW a
    robot moves: that is fully local to each robot's waypoint_nav_node
    (A* planning) and spatial_mutex (traffic negotiation).
    """

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

        self.goal_pubs = {}
        self.status_summary_pubs = {}
        for rid in ROBOT_IDS:
            self.goal_pubs[rid] = self.create_publisher(PoseStamped, f'/{rid}/goal_pose', qos)
            self.status_summary_pubs[rid] = self.create_publisher(
                String, f'/{rid}/mission_status_summary', qos
            )
            self.create_subscription(
                Odometry, f'/{rid}/odom',
                lambda msg, r=rid: self.odom_callback(msg, r),
                qos
            )
            self.create_subscription(
                String, f'/{rid}/health_status',
                lambda msg, r=rid: self.health_status_callback(msg, r),
                qos
            )
            self.create_subscription(
                String, f'/{rid}/task_complete',
                lambda msg, r=rid: self.task_complete_callback(msg, r),
                qos
            )

        self.charging_list_pub = self.create_publisher(String, '/fleet/charging_robots', qos)

        self.assignment_timer = self.create_timer(0.1, self.assignment_loop)
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_loop)

        self.get_logger().info(
            f"MissionController up (task allocation + watchdog only, no motion control). "
            f"Robots: {ROBOT_IDS} | Initial task_queue size: {len(self.task_queue)}"
        )

    def odom_callback(self, msg: Odometry, robot_id: str):
        worker = self.workers[robot_id]

        worker.x = msg.pose.pose.position.x
        worker.y = msg.pose.pose.position.y

        worker.has_odom = True
        worker.last_heartbeat = time.time()

        if worker.state == STATE_OFFLINE:
            worker.state = STATE_IDLE
            worker.current_task = None
            self.get_logger().info(f"[{robot_id}] odom resumed -> back to IDLE")

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
            self.mark_offline(robot_id, reason=reason or "self_reported")

    def pause_for_charge(self, robot_id: str, status: str, reason: str = ""):
        worker = self.workers[robot_id]

        if worker.state in PARKED_STATES:
            return

        self.get_logger().info(f"[{robot_id}] pausing for {status} (reason='{reason}')")
        worker.state = status

        # No cmd_vel publish here. spatial_mutex sees this same
        # health_status message independently and forces MUTEX_PARKED on
        # 'mutex_clearance', which waypoint_nav_node already halts on.

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
            return

        self.get_logger().info(f"[{robot_id}] resumed from {worker.state} -> IDLE")
        worker.state = STATE_IDLE
        worker.last_heartbeat = time.time()

    def watchdog_loop(self):
        now = time.time()
        charging_ids = []

        for robot_id, worker in self.workers.items():
            if worker.state == STATE_OFFLINE:
                continue

            if worker.state in PARKED_STATES:
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
        # No central cmd_vel publish. A robot with a dead heartbeat is
        # exactly the case where a central "stop" command is least likely
        # to be received anyway -- the local safety_fallback watchdog on
        # that robot is the correct layer for that.

        if worker.current_task is not None:
            self.get_logger().warn(
                f"[{robot_id}] re-queuing task {worker.current_task['task_id']} "
                f"at front of task_queue"
            )
            self.task_queue.insert(0, worker.current_task)
            worker.current_task = None

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

        goal = PoseStamped()
        goal.pose.position.x = task['x']
        goal.pose.position.y = task['y']
        self.goal_pubs[robot_id].publish(goal)

        self.get_logger().info(
            f"[{robot_id}] assigned task {task['task_id']} -> ({task['x']}, {task['y']}); "
            f"goal_pose published, motion is now entirely this robot's own decision"
        )

    def task_complete_callback(self, msg: String, robot_id: str):
        self.complete_task(robot_id)

    def complete_task(self, robot_id: str):
        worker = self.workers[robot_id]

        if worker.current_task is not None:
            self.get_logger().info(f"[{robot_id}] completed task {worker.current_task['task_id']}")
        worker.current_task = None
        worker.state = STATE_IDLE

    def assignment_loop(self):
        for robot_id, worker in self.workers.items():
            if worker.state == STATE_OFFLINE:
                continue
            if worker.state in PARKED_STATES:
                continue
            if worker.state == STATE_IDLE:
                self.assign_task_if_idle(robot_id)


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
