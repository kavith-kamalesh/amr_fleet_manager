import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json
import time

from amr_fleet_manager.robot_common import (
    MUTEX_CLEAR, MUTEX_WAIT, MUTEX_REROUTE, MUTEX_PARKED,
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

        self.robot_id = self.get_parameter('robot_id').value
        self.priority = self.get_parameter('priority').value
        self.reservation_buffer = self.get_parameter('reservation_buffer_sec').value
        self.reroute_wait_threshold = self.get_parameter('reroute_wait_threshold_sec').value
        self.peer_timeout = self.get_parameter('peer_timeout_sec').value

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

        self.timer = self.create_timer(0.1, self.mutex_loop)
        self.get_logger().info(f"SpatialMutex up for robot_id={self.robot_id}")

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
        try:
            data = json.loads(msg.data)
            peer_id = data['id']
            if peer_id == self.robot_id:
                return
            nodes = data['nodes']
        except (json.JSONDecodeError, KeyError, TypeError):
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

    def publish_current_intent_now(self):
        payload = json.dumps({
            'id': self.robot_id,
            'nodes': [list(n) for n in self.current_edge_nodes],
            'window': list(self.current_edge_window),
            'priority': self.priority,
        })
        self.broadcast_pub.publish(String(data=payload))

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
            if pid in self.known_charging_robots:
                del self.peer_intents[pid]
                continue
            if now - self.peer_intents[pid]['last_seen'] > self.peer_timeout:
                del self.peer_intents[pid]

    @staticmethod
    def windows_conflict(w1, w2):
        return not (w1[1] < w2[0] or w2[1] < w1[0])

    def peer_has_priority(self, pid, peer_priority):
        if peer_priority > self.priority:
            return True
        if peer_priority < self.priority:
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
            for pid, info in self.peer_intents.items():
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
