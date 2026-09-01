"""
Distributed Spatial Mutex (edge-side, decentralized).
Each robot runs one instance of this in its own namespace. It negotiates
intersection/edge locks PEER-TO-PEER over the peer_states channel (which
in hardware deployment rides over ESP-NOW) -- no central server involved.
This is the ROS2 port of the validated hybrid grid/graph reservation
logic from the 2D simulation (sim2d_hybrid.py), adapted for:
  - real network latency (larger reservation buffer than the sim used)
  - decentralized broadcast instead of a shared in-memory table
  - graceful operation if the central FMS is offline (mutex logic never
    depends on central_dispatcher.py being alive)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json
import time


class SpatialMutex(Node):
    def __init__(self):
        super().__init__('spatial_mutex')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('priority', 0.5)
        self.declare_parameter('reservation_buffer_sec', 0.6)  # wider than sim's 0.3s to absorb real network latency
        self.declare_parameter('reroute_wait_threshold_sec', 2.0)
        self.declare_parameter('peer_timeout_sec', 1.5)

        self.robot_id = self.get_parameter('robot_id').value
        self.priority = self.get_parameter('priority').value
        self.reservation_buffer = self.get_parameter('reservation_buffer_sec').value
        self.reroute_wait_threshold = self.get_parameter('reroute_wait_threshold_sec').value
        self.peer_timeout = self.get_parameter('peer_timeout_sec').value

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

        # Broadcasts this robot's current edge + time window to all peers
        self.broadcast_pub = self.create_publisher(String, '/fleet/spatial_intent', qos)
        # Listens to all peers' broadcasts (including its own, filtered out)
        self.create_subscription(String, '/fleet/spatial_intent', self.peer_intent_callback, qos)

        # Output: whether this robot is currently clear to proceed
        self.clearance_pub = self.create_publisher(String, 'mutex_clearance', qos)

        # Current edge this robot wants/has: (node_a, node_b) as strings, or None
        self.current_edge_nodes = None
        self.current_edge_window = None  # (t_start, t_end)

        self.peer_intents = {}  # peer_id -> {'nodes': set, 'window': (t0,t1), 'priority': float, 'last_seen': t}

        self.wait_start_time = None
        self.reroute_requested = False

        self.timer = self.create_timer(0.1, self.mutex_loop)
        self.get_logger().info(f"SpatialMutex up for robot_id={self.robot_id}")

    def set_intent(self, node_a, node_b, t_start, t_end):
        """Called externally (by the waypoint nav node) when this robot
        wants to claim/announce it's about to traverse edge (node_a,node_b)."""
        self.current_edge_nodes = frozenset([tuple(node_a), tuple(node_b)])
        self.current_edge_window = (t_start - self.reservation_buffer, t_end + self.reservation_buffer)
        self.wait_start_time = None
        self.reroute_requested = False

    def peer_intent_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            peer_id = data['id']
            if peer_id == self.robot_id:
                return
            nodes = frozenset(tuple(n) for n in data['nodes'])
            window = tuple(data['window'])
            peer_priority = data.get('priority', 0.5)
        except (json.JSONDecodeError, KeyError, TypeError):
            self.get_logger().warn("Malformed spatial_intent packet, dropping")
            return

        self.peer_intents[peer_id] = {
            'nodes': nodes, 'window': window,
            'priority': peer_priority, 'last_seen': time.time(),
        }

    def prune_stale_peers(self):
        now = time.time()
        for pid in list(self.peer_intents.keys()):
            if now - self.peer_intents[pid]['last_seen'] > self.peer_timeout:
                del self.peer_intents[pid]

    @staticmethod
    def windows_conflict(w1, w2):
        return not (w1[1] < w2[0] or w2[1] < w1[0])

    def mutex_loop(self):
        self.prune_stale_peers()

        # Broadcast our own intent every tick so peers have fresh data
        if self.current_edge_nodes is not None:
            payload = json.dumps({
                'id': self.robot_id,
                'nodes': [list(n) for n in self.current_edge_nodes],
                'window': list(self.current_edge_window),
                'priority': self.priority,
            })
            self.broadcast_pub.publish(String(data=payload))

        clearance = "CLEAR"
        blocked_by = None

        if self.current_edge_nodes is not None:
            for pid, info in self.peer_intents.items():
                if not (self.current_edge_nodes & info['nodes']):
                    continue  # no shared node, no conflict
                if self.windows_conflict(self.current_edge_window, info['window']):
                    if info['priority'] >= self.priority:
                        blocked_by = pid
                        break

        if blocked_by is not None:
            clearance = "WAIT"
            if self.wait_start_time is None:
                self.wait_start_time = time.time()
            elif time.time() - self.wait_start_time > self.reroute_wait_threshold:
                clearance = "REROUTE_REQUESTED"
                self.reroute_requested = True
        else:
            self.wait_start_time = None

        self.clearance_pub.publish(String(data=clearance))


def main(args=None):
    rclpy.init(args=args)
    node = SpatialMutex()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
