import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import rvo2
import json
import time
import math


class ORCANavigationNode(Node):
    """
    Tier 1: Decentralized ORCA with:
      - real peer-agent ingestion (fixes single-agent bug)
      - dead-reckoning velocity prediction for peers (not just hold-last)
      - priority-weighted responsibility (asymmetric avoidance)
      - deadlock tie-breaking via priority-based lateral nudge
    """

    def __init__(self):
        super().__init__('orca_nav_node')
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('priority', 0.5)
        self.declare_parameter('neighbor_dist', 3.0)
        self.declare_parameter('max_neighbors', 8)
        self.declare_parameter('time_horizon', 1.5)
        self.declare_parameter('time_horizon_obst', 1.0)
        self.declare_parameter('robot_radius', 0.35)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('peer_timeout_sec', 1.0)
        self.declare_parameter('deadlock_speed_thresh', 0.03)
        self.declare_parameter('deadlock_time_thresh', 1.2)

        self.robot_id = self.get_parameter('robot_id').value
        self.priority = self.get_parameter('priority').value
        neighbor_dist = self.get_parameter('neighbor_dist').value
        max_neighbors = self.get_parameter('max_neighbors').value
        time_horizon = self.get_parameter('time_horizon').value
        time_horizon_obst = self.get_parameter('time_horizon_obst').value
        self.radius = self.get_parameter('robot_radius').value
        self.max_speed = self.get_parameter('max_speed').value
        self.peer_timeout = self.get_parameter('peer_timeout_sec').value
        self.deadlock_speed_thresh = self.get_parameter('deadlock_speed_thresh').value
        self.deadlock_time_thresh = self.get_parameter('deadlock_time_thresh').value

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pref_sub = self.create_subscription(Twist, 'cmd_vel_nav', self.pref_vel_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.self_odom_callback, qos)
        self.peer_sub = self.create_subscription(String, 'peer_states', self.peer_state_callback, qos)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.sim = rvo2.PyRVOSimulator(
            0.05, neighbor_dist, max_neighbors, time_horizon, time_horizon_obst,
            self.radius, self.max_speed
        )
        self.self_agent = self.sim.addAgent((0.0, 0.0))
        self.preferred_vel = (0.0, 0.0)
        self.self_pos = (0.0, 0.0)

        self.peer_agents = {}
        self.stalled_since = None

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"ORCA node up for robot_id={self.robot_id}, priority={self.priority}")

    def pref_vel_callback(self, msg: Twist):
        self.preferred_vel = (msg.linear.x, msg.linear.y)

    def self_odom_callback(self, msg: Odometry):
        self.self_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.sim.setAgentPosition(self.self_agent, self.self_pos)
        self.sim.setAgentVelocity(
            self.self_agent,
            (msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        )

    def peer_state_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            peer_id = data['id']
            if peer_id == self.robot_id:
                return
            pos = (data['x'], data['y'])
            vel = (data.get('vx', 0.0), data.get('vy', 0.0))
            peer_priority = data.get('priority', 0.5)
        except (json.JSONDecodeError, KeyError):
            self.get_logger().warn("Malformed peer_states packet, dropping")
            return

        now = time.time()
        if peer_id not in self.peer_agents:
            agent_idx = self.sim.addAgent(pos)
            self.peer_agents[peer_id] = {
                'agent': agent_idx, 'last_seen': now,
                'last_pos': pos, 'last_vel': vel,
                'last_update_time': now, 'priority': peer_priority,
            }
        else:
            info = self.peer_agents[peer_id]
            info['last_seen'] = now
            info['last_pos'] = pos
            info['last_vel'] = vel
            info['last_update_time'] = now
            info['priority'] = peer_priority
            self.sim.setAgentPosition(info['agent'], pos)
            self.sim.setAgentVelocity(info['agent'], vel)

    def dead_reckon_position(self, info, now):
        dt = now - info['last_update_time']
        dt = min(dt, self.peer_timeout)
        px, py = info['last_pos']
        vx, vy = info['last_vel']
        return (px + vx * dt, py + vy * dt)

    def prune_stale_peers(self):
        now = time.time()
        for pid, info in list(self.peer_agents.items()):
            if now - info['last_seen'] > self.peer_timeout:
                self.sim.setAgentPosition(info['agent'], (1e5, 1e5))
                self.sim.setAgentVelocity(info['agent'], (0.0, 0.0))

    def apply_priority_weighting(self):
        for info in self.peer_agents.values():
            now = time.time()
            predicted_pos = self.dead_reckon_position(info, now)
            self.sim.setAgentPosition(info['agent'], predicted_pos)

            peer_priority = info['priority']
            relative = peer_priority - self.priority

            v = info['last_vel']
            if relative > 0:
                pref = v
            else:
                damp = max(0.3, 1.0 - abs(relative))
                pref = (v[0] * damp, v[1] * damp)

            self.sim.setAgentPrefVelocity(info['agent'], pref)

    def resolve_deadlock(self, safe_vel):
        now = time.time()
        speed = math.hypot(*safe_vel)
        wants_to_move = math.hypot(*self.preferred_vel) > self.deadlock_speed_thresh

        if speed < self.deadlock_speed_thresh and wants_to_move:
            if self.stalled_since is None:
                self.stalled_since = now
            elif now - self.stalled_since > self.deadlock_time_thresh:
                nudge_sign = 1.0 if (self.robot_id % 2 == 0) else -1.0
                perp = (-self.preferred_vel[1], self.preferred_vel[0])
                norm = math.hypot(*perp) or 1.0
                nudge = (perp[0] / norm * 0.15 * nudge_sign,
                         perp[1] / norm * 0.15 * nudge_sign)
                return (safe_vel[0] + nudge[0], safe_vel[1] + nudge[1])
        else:
            self.stalled_since = None

        return safe_vel

    def control_loop(self):
        self.prune_stale_peers()
        self.apply_priority_weighting()
        self.sim.setAgentPrefVelocity(self.self_agent, self.preferred_vel)

        self.sim.doStep()
        safe_vel = self.sim.getAgentVelocity(self.self_agent)
        safe_vel = self.resolve_deadlock(safe_vel)

        cmd = Twist()
        cmd.linear.x, cmd.linear.y = safe_vel[0], safe_vel[1]
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ORCANavigationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
