import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json
import time
import math

from amr_fleet_manager import nav_graph
from amr_fleet_manager.robot_common import (
    MUTEX_CLEAR, MUTEX_WAIT, MUTEX_REROUTE, MUTEX_PARKED,
    sign_payload, verify_payload,
)

PARKED_STATES = ("CHARGING", "SHIFT_CHANGE")


class SpatialMutex(Node):
    def __init__(self):
        super().__init__('spatial_mutex')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('priority', 0.5)
        self.declare_parameter('reservation_buffer_sec', 0.6)
        self.declare_parameter('reroute_wait_threshold_sec', 2.0)
        self.declare_parameter('peer_timeout_sec', 1.5)
        self.declare_parameter('hmac_key', 'sih26123-demo-preshared-key')
        self.declare_parameter('priority_aging_rate', 0.05)
        # Distance-radius neighbor filter: a peer whose reserved edge is
        # farther than this from our own current edge is skipped entirely
        # before any conflict check. Validated in benchmark_scalability.py
        # (4x grid CELL, i.e. 8.0m at the default CELL=2.0) -- 55-72%
        # fewer conflict-check comparisons at every tested fleet size
        # (3/6/12/24 robots), with IDENTICAL makespan/timeout/collision
        # outcomes to the unfiltered version. This does not reduce
        # broadcast message volume (every robot still publishes to
        # everyone) -- it only reduces what each robot evaluates.
        self.declare_parameter('neighbor_radius_m', 4 * nav_graph.CELL)

        self.robot_id = self.get_parameter('robot_id').value
        self.base_priority = self.get_parameter('priority').value
        self.reservation_buffer = self.get_parameter('reservation_buffer_sec').value
        self.reroute_wait_threshold = self.get_parameter('reroute_wait_threshold_sec').value
        self.peer_timeout = self.get_parameter('peer_timeout_sec').value
        self.hmac_key = self.get_parameter('hmac_key').value
        self.priority_aging_rate = self.get_parameter('priority_aging_rate').value
        self.neighbor_radius_m = self.get_parameter('neighbor_radius_m').value

        if self.hmac_key == 'sih26123-demo-preshared-key':
            self.get_logger().warn(
                "hmac_key left at its default value -- override it via the "
                "launch file / params yaml so all robots share the SAME key. "
                "Every robot using the default independently is not secure, "
                "it's just obscurity."
            )

        qos_active = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                 history=HistoryPolicy.KEEP_LAST, depth=10)

        qos_background = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                     history=HistoryPolicy.KEEP_LAST, depth=1)

        self.broadcast_pub = self.create_publisher(String, '/fleet/spatial_intent', qos_active)
        self.create_subscription(String, '/fleet/spatial_intent', self.peer_intent_callback, qos_active)
        self.clearance_pub = self.create_publisher(String, 'mutex_clearance', qos_active)

        self.create_subscription(String, 'planned_edge', self.planned_edge_callback, qos_active)

        self.create_subscription(String, '/fleet/spatial_intent', self.background_peer_listener, qos_background)
        self.background_position_log = {}
        self.last_background_update_time = None

        self.create_subscription(String, '/fleet/charging_robots', self.charging_list_callback, qos_active)
        self.known_charging_robots = set()

        self.create_subscription(String, 'health_status', self.self_health_callback, qos_active)
        self.self_state = "IDLE"
        self.charge_start_time = None

        self.costmap_seed_pub = self.create_publisher(String, 'costmap_seed', qos_active)
        self.sync_request_pub = self.create_publisher(String, '/fleet/state_sync_request', qos_active)
        self.create_subscription(String, '/fleet/state_sync_request', self.state_sync_request_callback, qos_active)

        self.current_edge_nodes = None
        self.current_edge_window = None
        self.wait_start_time = None
        self.reroute_requested = False

        self.peer_intents = {}
        self.peer_last_seq = {}
        self.out_seq = 0

        self.timer = self.create_timer(0.1, self.mutex_loop)
        self.get_logger().info(
            f"SpatialMutex up for robot_id={self.robot_id} "
            f"(HMAC-signed intents, priority aging, neighbor_radius_m={self.neighbor_radius_m})"
        )

    def effective_priority(self):
        if self.wait_start_time is None:
            return self.base_priority
        waited = time.time() - self.wait_start_time
        return self.base_priority + self.priority_aging_rate * waited

    def planned_edge_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            n1, n2 = data['nodes']
            t_start = data['t_start']
            t_end = data['t_end']
        except (json.JSONDecodeError, KeyError, ValueError, TypeError):
            self.get_logger().warn("Malformed planned_edge packet, dropping")
            return

        if self.self_state in PARKED_STATES:
            return

        self.set_intent(tuple(n1), tuple(n2), t_start, t_end)

    def self_health_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        new_status = data.get('status')
        if new_status in PARKED_STATES and self.self_state not in PARKED_STATES:
            self.enter_low_power_mode(new_status)
        elif new_status == "IDLE" and self.self_state in PARKED_STATES:
            self.exit_low_power_mode()

        if new_status:
            self.self_state = new_status

    def enter_low_power_mode(self, status: str):
        self.charge_start_time = time.time()
        self.current_edge_nodes = None
        self.wait_start_time = None
        self.clearance_pub.publish(String(data=MUTEX_PARKED))
        self.get_logger().info(
            f"Entering low-power state ({status}): suspending spatial_intent "
            f"broadcast and active conflict computation. Background listener stays on."
        )

    def exit_low_power_mode(self):
        self.get_logger().info("Resuming from low-power state -> rebuilding awareness before moving.")

        self.publish_costmap_seed()

        if (self.last_background_update_time is None or
                self.charge_start_time is None or
                self.last_background_update_time < self.charge_start_time):
            self.get_logger().warn(
                "No peer updates received during charging window -- "
                "possible Wi-Fi drop. Requesting fleet-wide state resync."
            )
            self.request_state_sync()

        self.charge_start_time = None

    def background_peer_listener(self, msg: String):
        payload = self._parse_and_verify(msg.data, context="background")
        if payload is None:
            return
        peer_id = payload.get('id')
        if peer_id == self.robot_id:
            return
        nodes = payload.get('nodes')
        if nodes is None:
            return

        now = time.time()
        self.background_position_log[peer_id] = {'nodes': nodes, 'last_seen': now}
        self.last_background_update_time = now

    def publish_costmap_seed(self):
        payload = json.dumps({
            'robot_id': self.robot_id,
            'seed_source': 'background_low_power_log',
            'peer_positions': self.background_position_log,
            'timestamp': time.time(),
        })
        self.costmap_seed_pub.publish(String(data=payload))
        self.get_logger().info(
            f"Published costmap seed with {len(self.background_position_log)} "
            f"known peer positions."
        )

    def request_state_sync(self):
        payload = json.dumps({'requesting_robot': self.robot_id, 'timestamp': time.time()})
        self.sync_request_pub.publish(String(data=payload))

    def state_sync_request_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            requester = data.get('requesting_robot')
        except json.JSONDecodeError:
            return

        if requester == self.robot_id:
            return
        if self.self_state in PARKED_STATES:
            return
        if self.current_edge_nodes is None:
            return

        self.publish_current_intent_now()

    def _next_seq(self):
        self.out_seq += 1
        return self.out_seq

    def _sign_and_publish(self, publisher, payload: dict):
        sig = sign_payload(payload, self.hmac_key)
        publisher.publish(String(data=json.dumps({**payload, 'sig': sig})))

    def _parse_and_verify(self, raw: str, context: str):
        try:
            data = json.loads(raw)
            sig = data.pop('sig', None)
            peer_id = data['id']
        except (json.JSONDecodeError, KeyError, TypeError):
            self.get_logger().warn(f"Malformed spatial_intent packet ({context}), dropping")
            return None

        if peer_id == self.robot_id:
            return None

        if not verify_payload(data, sig, self.hmac_key):
            self.get_logger().warn(
                f"REJECTED spatial_intent from id={peer_id} ({context}): "
                f"bad or missing HMAC signature -- possible spoofed peer."
            )
            return None

        return data

    def charging_list_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.known_charging_robots = set(data.get('charging_robots', []))
        except json.JSONDecodeError:
            pass

    def set_intent(self, node_a, node_b, t_start, t_end):
        self.current_edge_nodes = frozenset([tuple(node_a), tuple(node_b)])
        self.current_edge_window = (t_start - self.reservation_buffer, t_end + self.reservation_buffer)
        self.wait_start_time = None
        self.reroute_requested = False

    def peer_intent_callback(self, msg: String):
        payload = self._parse_and_verify(msg.data, context="active")
        if payload is None:
            return

        peer_id = payload['id']
        seq = payload.get('seq')

        if seq is None:
            self.get_logger().warn(f"spatial_intent from id={peer_id} missing seq, dropping")
            return

        last_seq = self.peer_last_seq.get(peer_id)
        if last_seq is not None and seq <= last_seq:
            self.get_logger().warn(
                f"REJECTED spatial_intent from id={peer_id}: seq={seq} <= last accepted "
                f"seq={last_seq} -- replay or out-of-order delivery."
            )
            return
        self.peer_last_seq[peer_id] = seq

        try:
            nodes = frozenset(tuple(n) for n in payload['nodes'])
            window = tuple(payload['window'])
            peer_priority = payload.get('priority', 0.5)
        except (KeyError, TypeError):
            self.get_logger().warn(f"spatial_intent from id={peer_id} missing fields, dropping")
            return

        self.peer_intents[peer_id] = {
            'nodes': nodes, 'window': window,
            'priority': peer_priority, 'last_seen': time.time(),
        }

    def prune_stale_peers(self):
        now = time.time()
        for pid in list(self.peer_intents.keys()):
            if pid in self.known_charging_robots:
                del self.peer_intents[pid]
                continue
            if now - self.peer_intents[pid]['last_seen'] > self.peer_timeout:
                del self.peer_intents[pid]

    @staticmethod
    def windows_conflict(w1, w2):
        return not (w1[1] < w2[0] or w2[1] < w1[0])

    @staticmethod
    def _edge_midpoint(nodes_frozenset):
        positions = [(n[0] * nav_graph.CELL, n[1] * nav_graph.CELL) for n in nodes_frozenset]
        mx = sum(p[0] for p in positions) / len(positions)
        my = sum(p[1] for p in positions) / len(positions)
        return (mx, my)

    @staticmethod
    def _distance(p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def peer_has_priority(self, pid, peer_priority):
        """True if the peer should go first. Both sides publish their
        effective (aged) priority, so this comparison is symmetric and a
        robot that has waited long enough always eventually wins."""
        self_eff = self.effective_priority()
        if peer_priority > self_eff:
            return True
        if peer_priority < self_eff:
            return False
        return pid < self.robot_id

    def mutex_loop(self):
        if self.self_state in PARKED_STATES:
            return

        self.prune_stale_peers()

        if self.current_edge_nodes is not None:
            self.publish_current_intent_now()

        clearance = MUTEX_CLEAR
        blocked_by = None

        if self.current_edge_nodes is not None:
            self_midpoint = self._edge_midpoint(self.current_edge_nodes)
            for pid, info in self.peer_intents.items():
                peer_midpoint = self._edge_midpoint(info['nodes'])
                if self._distance(self_midpoint, peer_midpoint) > self.neighbor_radius_m:
                    continue
                if not (self.current_edge_nodes & info['nodes']):
                    continue
                if self.windows_conflict(self.current_edge_window, info['window']):
                    if self.peer_has_priority(pid, info['priority']):
                        blocked_by = pid
                        break

        if blocked_by is not None:
            clearance = MUTEX_WAIT
            if self.wait_start_time is None:
                self.wait_start_time = time.time()
            elif time.time() - self.wait_start_time > self.reroute_wait_threshold:
                clearance = MUTEX_REROUTE
                self.reroute_requested = True
        else:
            self.wait_start_time = None

        self.clearance_pub.publish(String(data=clearance))

    def publish_current_intent_now(self):
        payload = {
            'id': self.robot_id,
            'nodes': [list(n) for n in self.current_edge_nodes],
            'window': list(self.current_edge_window),
            'priority': self.effective_priority(),
            'seq': self._next_seq(),
        }
        self._sign_and_publish(self.broadcast_pub, payload)


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
