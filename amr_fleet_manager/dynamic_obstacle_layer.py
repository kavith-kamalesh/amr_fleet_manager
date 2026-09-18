"""
Dynamic obstacle sensing + fleet-shared clearing.

PROBLEM THIS SOLVES: a robot's LiDAR sees a box in the aisle, the fleet
needs to route around it -- and later, when the box is moved, every robot
(not just the one that happens to drive past again) needs to find out it's
gone. Nothing else in this repo does either half of this; costmap_seed in
spatial_mutex.py is peer-position gossip, not obstacle tracking.

TWO SEPARATE CLEARING PATHS, deliberately not merged into one:

  1. ACTIVE clearing (trustworthy): this robot's own LiDAR beam passes
     THROUGH a cell it previously reported occupied and the return comes
     back farther away (or no return at all). That's a direct physical
     observation the obstacle is gone -- same principle as raytracing in a
     real occupancy grid, simplified here to "check the range reading at
     that cell's bearing" since this repo has no persistent costmap grid.

  2. PASSIVE expiry (weaker, fallback only): nobody has driven past a
     known obstacle cell in OBSTACLE_TTL_SEC, so it ages out rather than
     blocking a corridor forever. This is NOT the same strength of
     evidence as (1) -- it just means "we don't know anymore", not
     "confirmed clear". Logged separately on purpose; if asked live,
     say so plainly rather than implying both paths are equally certain.

Broadcasts/consumes JSON on /fleet/dynamic_obstacles:
    {'reporter_id', 'cell': [cx, cy], 'state': 'OCCUPIED'|'CLEARED',
     'seq', 'timestamp'}

Ordering, same pattern as spatial_mutex.py's spatial_intent channel:
  - seq is monotonic PER REPORTER -- used to drop out-of-order/duplicate
    packets from that one sender.
  - timestamp is used to merge across DIFFERENT reporters about the same
    cell (last write wins), since seq counters aren't comparable across
    senders.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class DynamicObstacleLayer(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_layer')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('cell_size', 0.3)             # grid resolution, meters
        self.declare_parameter('detection_range', 3.0)        # ignore returns beyond this
        self.declare_parameter('confirm_count', 3)            # consecutive hits to report OCCUPIED
        self.declare_parameter('clear_confirm_count', 2)       # consecutive misses to report CLEARED
        self.declare_parameter('obstacle_ttl_sec', 20.0)      # passive-expiry fallback

        self.robot_id = self.get_parameter('robot_id').value
        self.cell_size = self.get_parameter('cell_size').value
        self.detection_range = self.get_parameter('detection_range').value
        self.confirm_count = self.get_parameter('confirm_count').value
        self.clear_confirm_count = self.get_parameter('clear_confirm_count').value
        self.obstacle_ttl = self.get_parameter('obstacle_ttl_sec').value

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=20)

        self.x = self.y = self.yaw = 0.0
        self.has_pose = False

        self.create_subscription(Odometry, 'odom', self.odom_cb, qos)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, qos)
        self.create_subscription(String, '/fleet/dynamic_obstacles', self.peer_obstacle_cb, qos)
        self.obstacle_pub = self.create_publisher(String, '/fleet/dynamic_obstacles', qos)

        self._candidate_hits = {}     # cell -> consecutive-detection count (not yet confirmed)
        self._clear_hits = {}         # cell -> consecutive-miss count (confirmed obstacle, being un-confirmed)
        self.known_obstacles = {}     # cell -> {'state','reporter_id','timestamp','seq'} -- fleet-shared view
        self._last_seq_from = {}      # reporter_id -> last accepted seq (ordering, not merging)
        self._out_seq = 0

        self.create_timer(1.0, self._prune_expired)
        self.get_logger().info(f"DynamicObstacleLayer up for robot_id={self.robot_id}")

    # ---------------- pose ----------------

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.has_pose = True

    def _to_cell(self, px, py):
        return (round(px / self.cell_size), round(py / self.cell_size))

    def _cell_center(self, cell):
        cx, cy = cell
        return (cx * self.cell_size, cy * self.cell_size)

    # ---------------- broadcasting ----------------

    def _broadcast(self, cell, state):
        payload = json.dumps({
            'reporter_id': self.robot_id,
            'cell': list(cell),
            'state': state,
            'seq': self._out_seq,
            'timestamp': time.time(),
        })
        self._out_seq += 1
        self.obstacle_pub.publish(String(data=payload))
        self._apply_local(self.robot_id, cell, state, time.time())

    def _apply_local(self, reporter_id, cell, state, timestamp):
        prev = self.known_obstacles.get(cell)
        if state == 'CLEARED':
            if prev is not None:
                del self.known_obstacles[cell]
            return
        if prev is None or timestamp >= prev.get('timestamp', 0):
            self.known_obstacles[cell] = {
                'state': state, 'reporter_id': reporter_id, 'timestamp': timestamp,
            }

    # ---------------- own LiDAR: detect + actively clear ----------------

    def scan_cb(self, msg: LaserScan):
        if not self.has_pose or not msg.ranges:
            return

        # --- 1. Detect new obstacles from raw returns ---
        seen_this_scan = set()
        for i, r in enumerate(msg.ranges):
            if r != r or r <= 0.05 or r > self.detection_range:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            world_angle = self.yaw + angle
            px = self.x + r * math.cos(world_angle)
            py = self.y + r * math.sin(world_angle)
            cell = self._to_cell(px, py)
            seen_this_scan.add(cell)

            if cell in self.known_obstacles and self.known_obstacles[cell]['state'] == 'OCCUPIED':
                continue  # already confirmed -- handled by the clearing pass below
            self._candidate_hits[cell] = self._candidate_hits.get(cell, 0) + 1
            if self._candidate_hits[cell] >= self.confirm_count:
                del self._candidate_hits[cell]
                self.get_logger().info(f"Obstacle CONFIRMED at cell {cell}")
                self._broadcast(cell, 'OCCUPIED')

        # candidate cells not re-seen this scan lose their streak -- avoids
        # a single noisy blip promoting to a confirmed obstacle
        for cell in list(self._candidate_hits.keys()):
            if cell not in seen_this_scan:
                del self._candidate_hits[cell]

        # --- 2. Actively re-check known obstacles that are in range/bearing now ---
        for cell, info in list(self.known_obstacles.items()):
            if info['state'] != 'OCCUPIED':
                continue
            cx, cy = self._cell_center(cell)
            dx, dy = cx - self.x, cy - self.y
            expected_dist = math.hypot(dx, dy)
            if expected_dist > self.detection_range:
                continue  # out of sensing range -- can't confirm or clear right now

            bearing_world = math.atan2(dy, dx)
            bearing_robot = bearing_world - self.yaw
            bearing_robot = (bearing_robot + math.pi) % (2 * math.pi) - math.pi
            if not (msg.angle_min <= bearing_robot <= msg.angle_max):
                continue  # not currently facing it

            idx = int(round((bearing_robot - msg.angle_min) / msg.angle_increment))
            if not (0 <= idx < len(msg.ranges)):
                continue
            actual = msg.ranges[idx]

            beam_passes_through = (actual != actual) or (actual > expected_dist + self.cell_size)
            if beam_passes_through:
                self._clear_hits[cell] = self._clear_hits.get(cell, 0) + 1
                if self._clear_hits[cell] >= self.clear_confirm_count:
                    del self._clear_hits[cell]
                    self.get_logger().info(f"Obstacle at cell {cell} ACTIVELY CONFIRMED CLEAR")
                    self._broadcast(cell, 'CLEARED')
            else:
                self._clear_hits[cell] = 0  # still there -- reset any partial clear streak

    # ---------------- fleet-shared obstacle map ----------------

    def peer_obstacle_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            reporter_id = data['reporter_id']
            if reporter_id == self.robot_id:
                return
            cell = tuple(data['cell'])
            state = data['state']
            seq = data.get('seq', 0)
            timestamp = data.get('timestamp', time.time())
        except (json.JSONDecodeError, KeyError, TypeError):
            self.get_logger().warn("Malformed dynamic_obstacles packet, dropping")
            return

        last_seq = self._last_seq_from.get(reporter_id, -1)
        if seq <= last_seq:
            return
        self._last_seq_from[reporter_id] = seq

        self._apply_local(reporter_id, cell, state, timestamp)

    # ---------------- passive expiry fallback ----------------

    def _prune_expired(self):
        now = time.time()
        for cell, info in list(self.known_obstacles.items()):
            if info['state'] == 'OCCUPIED' and (now - info['timestamp']) > self.obstacle_ttl:
                self.get_logger().warn(
                    f"Obstacle at cell {cell} UNCONFIRMED EXPIRED after {self.obstacle_ttl:.0f}s "
                    f"with no re-observation -- removing from map. This is NOT the same as an "
                    f"active LiDAR confirmation; treat routes through it cautiously."
                )
                del self.known_obstacles[cell]


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleLayer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
