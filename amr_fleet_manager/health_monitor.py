"""
Composite robot health scoring, shared fleet-wide P2P (same broadcast
pattern as spatial_mutex.py -- no central server).

Score in [0,1], weighted from four signals this repo already produces:
  battery         35%  -- from battery_monitor.py
  heartbeat_age   25%  -- freshness of own odom (proxy for local health)
  lidar_quality   20%  -- fraction of valid (non-NaN, in-range) returns
  stability       20%  -- inverse of recent reroute/stall frequency

Below LOW_HEALTH_THRESHOLD: publishes a degraded-status broadcast so
peers and mission_controller can react.
Below CRITICAL_HEALTH_THRESHOLD: publishes an emergency_replacement
request AND a manager_alert -- see alert_dispatcher.py for the
external-notification half of this.
"""

import time
import json
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


LOW_HEALTH_THRESHOLD = 0.45
CRITICAL_HEALTH_THRESHOLD = 0.25
STALL_WINDOW_SEC = 120.0


class HealthMonitor(Node):
    def __init__(self):
        super().__init__('health_monitor')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('peer_timeout_sec', 3.0)

        self.robot_id = self.get_parameter('robot_id').value
        self.peer_timeout = self.get_parameter('peer_timeout_sec').value

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

        self.battery = 1.0
        self.last_odom_time = None
        self.lidar_quality = 1.0
        self.reroute_events = deque()

        self.create_subscription(Float32, 'battery', self.battery_cb, qos)
        self.create_subscription(Odometry, 'odom', self.odom_cb, qos)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, qos)
        self.create_subscription(String, 'mutex_clearance', self.clearance_cb, qos)

        self.create_subscription(String, '/fleet/robot_health', self.peer_health_cb, qos)
        self.health_pub = self.create_publisher(String, '/fleet/robot_health', qos)
        self.replacement_pub = self.create_publisher(String, '/fleet/emergency_replacement', qos)
        self.alert_pub = self.create_publisher(String, '/fleet/manager_alert', qos)

        self.peer_health = {}
        self._last_state = "OK"
        self._replacement_sent = False
        self.health_seq = 0  # outgoing monotonic counter, see peer_health_cb

        self.create_timer(1.0, self.compute_and_broadcast)
        self.get_logger().info(f"HealthMonitor up | robot={self.robot_id}")

    def battery_cb(self, msg: Float32):
        self.battery = msg.data

    def odom_cb(self, msg: Odometry):
        self.last_odom_time = time.time()

    def scan_cb(self, msg: LaserScan):
        if not msg.ranges:
            return
        valid = sum(1 for r in msg.ranges
                    if r == r and msg.range_min <= r <= msg.range_max)
        self.lidar_quality = valid / len(msg.ranges)

    def clearance_cb(self, msg: String):
        # Clearance arrives as signed JSON {"state","ts","signature"}; the nav node
        # is the verifier, this node only counts events (trusted local topic).
        try:
            state = json.loads(msg.data).get("state", "")
        except (json.JSONDecodeError, AttributeError):
            state = msg.data
        if state == "REROUTE_REQUESTED":
            self.reroute_events.append(time.time())

    def peer_health_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            if d['robot_id'] == self.robot_id:
                return
            seq = d.get('seq', 0)
        except (json.JSONDecodeError, KeyError):
            return

        prev = self.peer_health.get(d['robot_id'])
        if prev is not None and seq <= prev.get('seq', -1):
            return

        self.peer_health[d['robot_id']] = {'score': d['score'], 'last_seen': time.time(), 'seq': seq}

    def _prune_reroutes(self, now):
        while self.reroute_events and now - self.reroute_events[0] > STALL_WINDOW_SEC:
            self.reroute_events.popleft()

    def compute_score(self):
        now = time.time()
        self._prune_reroutes(now)

        heartbeat_score = 1.0
        if self.last_odom_time is not None:
            age = now - self.last_odom_time
            heartbeat_score = max(0.0, 1.0 - age / 5.0)

        stability_score = max(0.0, 1.0 - len(self.reroute_events) / 3.0)

        score = (0.35 * self.battery +
                 0.25 * heartbeat_score +
                 0.20 * self.lidar_quality +
                 0.20 * stability_score)
        return round(max(0.0, min(1.0, score)), 4), {
            'battery': round(self.battery, 3),
            'heartbeat': round(heartbeat_score, 3),
            'lidar': round(self.lidar_quality, 3),
            'stability': round(stability_score, 3),
        }

    def _find_healthiest_idle_peer(self):
        now = time.time()
        candidates = [(pid, info['score']) for pid, info in self.peer_health.items()
                      if now - info['last_seen'] < self.peer_timeout]
        if not candidates:
            return None
        return max(candidates, key=lambda x: x[1])

    def compute_and_broadcast(self):
        score, breakdown = self.compute_score()

        state = ("CRITICAL" if score < CRITICAL_HEALTH_THRESHOLD else
                  "LOW" if score < LOW_HEALTH_THRESHOLD else "OK")

        self.health_pub.publish(String(data=json.dumps({
            'robot_id': self.robot_id,
            'score': score,
            'state': state,
            'breakdown': breakdown,
            'timestamp': time.time(),
            'seq': self.health_seq,
        })))
        self.health_seq += 1

        if state == "CRITICAL" and self._last_state != "CRITICAL":
            self._fire_emergency(score, breakdown)
        elif state != "CRITICAL":
            self._replacement_sent = False

        if state != self._last_state:
            self.get_logger().warn(f"Health state {self._last_state} -> {state} (score={score})")
            self.alert_pub.publish(String(data=json.dumps({
                'robot_id': self.robot_id, 'severity': state, 'score': score,
                'breakdown': breakdown, 'timestamp': time.time(),
                'message': f"Robot {self.robot_id} health is {state} (score {score:.2f})",
            })))

        self._last_state = state

    def _fire_emergency(self, score, breakdown):
        if self._replacement_sent:
            return
        replacement = self._find_healthiest_idle_peer()
        self.replacement_pub.publish(String(data=json.dumps({
            'failing_robot': self.robot_id,
            'failing_score': score,
            'breakdown': breakdown,
            'suggested_replacement': replacement[0] if replacement else None,
            'replacement_score': replacement[1] if replacement else None,
            'timestamp': time.time(),
        })))
        self._replacement_sent = True
        self.get_logger().error(
            f"EMERGENCY: robot {self.robot_id} score={score:.2f} < {CRITICAL_HEALTH_THRESHOLD}. "
            f"Suggested replacement: {replacement[0] if replacement else 'NONE AVAILABLE'}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
