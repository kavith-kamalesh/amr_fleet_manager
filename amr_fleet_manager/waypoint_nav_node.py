import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from amr_fleet_manager.secure_mutex_wrapper import verify_mutex_signature

CELL_M = 1.0
SAMPLE_STEP_M = 0.25


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class WaypointNavNode(Node):
    def __init__(self):
        super().__init__('waypoint_nav_node')
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('target_x', 4.0)
        self.declare_parameter('target_y', 2.0)
        self.declare_parameter('bypass_threshold_sec', 3.0)
        self.declare_parameter('max_linear', 0.2)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('spawn_offset_x', 0.0)
        self.declare_parameter('spawn_offset_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('lookahead_m', 2.0)
        self.declare_parameter('footprint_margin_m', 0.4)
        self.declare_parameter('reroute_offset_m', 1.2)
        self.declare_parameter('clearance_timeout_sec', 1.0)
        self.declare_parameter('start_active', False)

        gp = lambda n: self.get_parameter(n).value
        self.robot_id = int(gp('robot_id'))
        self.max_linear = gp('max_linear')
        self.max_angular = gp('max_angular')
        self.off_x, self.off_y = gp('spawn_offset_x'), gp('spawn_offset_y')
        self.tol = gp('goal_tolerance')
        self.lookahead = gp('lookahead_m')
        self.margin = gp('footprint_margin_m')
        self.reroute_offset = gp('reroute_offset_m')
        self.clearance_timeout = gp('clearance_timeout_sec')

        self.goal = (gp('target_x'), gp('target_y')) if gp('start_active') else None
        self.via = None
        self.rerouted = False
        self.x = self.y = self.yaw = 0.0
        self.has_pose = False
        self.clearance = "WAIT"
        self.last_clearance_t = None
        self._status = None
        self._idle_announced = False

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, qos)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, qos)
        self.create_subscription(String, 'mutex_clearance', self.mutex_callback, qos)
        self.publisher_cmd = self.create_publisher(Twist, 'cmd_vel_nav', qos)
        self.intent_pub = self.create_publisher(String, 'planned_intent', qos)
        self.status_pub = self.create_publisher(String, 'nav_status', qos)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Zero-Trust Nav up: verified+fresh clearance required, intent published.")

    def _target(self):
        return self.via if self.via is not None else self.goal

    def _cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.publisher_cmd.publish(t)

    def _set_status(self, s):
        if s != self._status:
            self._status = s
            self.status_pub.publish(String(data=s))

    def odom_callback(self, msg: Odometry):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        self.x, self.y = p.x + self.off_x, p.y + self.off_y
        self.has_pose = True

    def goal_callback(self, msg: PoseStamped):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.via = None
        self.rerouted = False
        self.get_logger().info(f"New goal {self.goal}")

    def mutex_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            state = payload.get("state", "")
            ok = verify_mutex_signature(state, payload.get("signature", ""), payload.get("ts"))
        except (json.JSONDecodeError, AttributeError):
            self.get_logger().warn("SECURITY: dropping unsigned/plaintext mutex command.",
                                   throttle_duration_sec=2.0)
            return
        if not ok:
            self.get_logger().error("SECURITY: invalid or stale signature, dropping mutex command.",
                                    throttle_duration_sec=2.0)
            return
        self.clearance = state
        self.last_clearance_t = time.monotonic()
        if state == "REROUTE_REQUESTED" and not self.rerouted and self.goal and self.has_pose:
            self._start_detour()

    def _start_detour(self):
        gx, gy = self.goal
        dx, dy = gx - self.x, gy - self.y
        d = math.hypot(dx, dy)
        if d < 0.3:
            return
        ux, uy = dx / d, dy / d
        side = 1.0 if self.robot_id % 2 == 0 else -1.0
        self.via = (self.x + ux * d * 0.5 - uy * side * self.reroute_offset,
                    self.y + uy * d * 0.5 + ux * side * self.reroute_offset)
        self.rerouted = True
        self.get_logger().warn(f"Detour via ({self.via[0]:.2f},{self.via[1]:.2f}); goal {self.goal} unchanged")

    def _plan_cells(self):
        pts = [(self.x, self.y)]
        px, py = self.x, self.y
        remaining = self.lookahead
        legs = [self._target()] + ([self.goal] if self.via is not None else [])
        for wx, wy in legs:
            seg = math.hypot(wx - px, wy - py)
            if seg < 1e-6:
                continue
            take = min(seg, remaining)
            n = max(1, int(take / SAMPLE_STEP_M))
            for k in range(1, n + 1):
                f = (take * k / n) / seg
                pts.append((px + (wx - px) * f, py + (wy - py) * f))
            remaining -= take
            if remaining <= 1e-6:
                break
            px, py = wx, wy
        cells = set()
        for x, y in pts:
            for dx in (-self.margin, self.margin):
                for dy in (-self.margin, self.margin):
                    cells.add((math.floor((x + dx) / CELL_M), math.floor((y + dy) / CELL_M)))
        return sorted(cells), self.lookahead - remaining

    def _publish_intent(self):
        now = time.time()
        cells, length = self._plan_cells()
        t1 = now + length / max(self.max_linear, 0.05) + 1.0
        self.intent_pub.publish(String(data=json.dumps({'cells': cells, 'window': [now, t1]})))
        self._idle_announced = False

    def _announce_idle(self):
        if not self._idle_announced:
            now = time.time()
            self.intent_pub.publish(String(data=json.dumps({'cells': [], 'window': [now, now]})))
            self._idle_announced = True

    def control_loop(self):
        if not self.has_pose or self.goal is None:
            self._cmd(0.0, 0.0)
            self._announce_idle()
            if self._status != "ARRIVED":
                self._set_status("IDLE")
            return

        tx, ty = self._target()
        dist = math.hypot(tx - self.x, ty - self.y)
        if dist < self.tol:
            if self.via is not None:
                self.via = None
                return
            self.get_logger().info("Goal reached")
            self.goal = None
            self._cmd(0.0, 0.0)
            self._announce_idle()
            self._set_status("ARRIVED")
            return

        self._publish_intent()

        stale = (self.last_clearance_t is None or
                 time.monotonic() - self.last_clearance_t > self.clearance_timeout)
        if stale:
            self._cmd(0.0, 0.0)
            self._set_status("WAITING")
            self.get_logger().warn("No fresh verified clearance; holding.", throttle_duration_sec=2.0)
            return
        if self.clearance != "CLEAR":
            self._cmd(0.0, 0.0)
            self._set_status("WAITING")
            return

        err = wrap_pi(math.atan2(ty - self.y, tx - self.x) - self.yaw)
        if abs(err) > 0.8:
            self._cmd(0.02, clamp(2.0 * err, -self.max_angular, self.max_angular))
        else:
            self._cmd(clamp(0.8 * dist, 0.05, self.max_linear),
                      clamp(1.5 * err, -self.max_angular, self.max_angular))
        self._set_status("REROUTING" if self.via is not None else "MOVING")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
