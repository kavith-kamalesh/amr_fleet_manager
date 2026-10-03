"""
pibt_fleet_node.py -- ROS 2 wrapper around fleet_protocol.FleetAgent.

One instance per robot (run it inside the robot's namespace). All coordination
logic is in FleetAgent (pure Python, unit-tested + stress-tested in
protocol_sim.py); this node only does I/O:

  subscribes  odom        nav_msgs/Odometry      -> which grid cell am I in?
              goal_pose   geometry_msgs/PoseStamped  task goal (world metres)
              /fleet/pibt std_msgs/String (JSON) -> peer STATE / INTENT messages
  publishes   /fleet/pibt std_msgs/String (JSON) -> my STATE / INTENT messages
              nav_waypoint geometry_msgs/PoseStamped -> centre of the next cell
              pibt_status std_msgs/String (JSON) -> per-tick status (dashboard)

Wiring: remap waypoint_nav_node's `goal_pose` to `nav_waypoint` (see
launch/pibt_fleet_bringup.launch.py) so the existing driver executes one cell
per tick.

TIME: every robot derives the tick from the shared ROS clock
(tick = floor(now / tick_period)). In Gazebo use use_sim_time:=true. On real
hardware the robots' clocks must be synchronised (chrony/NTP, or PTP) to well
under round_period (default 150 ms).

SAFETY: this node plans cell moves; it is NOT a collision-avoidance layer.
Keep safety_fallback / LiDAR e-stop running underneath.
"""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from amr_fleet_manager.fleet_protocol import FleetAgent
from amr_fleet_manager.pibt_core import grid_graph


class PibtFleetNode(Node):
    def __init__(self):
        super().__init__('pibt_fleet_node')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('priority', 0.5)
        self.declare_parameter('spawn_offset_x', 0.0)   # same meaning as waypoint_nav_node
        self.declare_parameter('spawn_offset_y', 0.0)
        self.declare_parameter('cell_size', 1.0)         # metres per grid cell
        self.declare_parameter('origin_x', 0.0)          # world position of cell (0,0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('grid_w', 12)
        self.declare_parameter('grid_h', 12)
        self.declare_parameter('blocked_cells_json', '[]')   # e.g. "[[1,1],[2,1]]"
        # {"1": {"cell": [0,0], "prio": 0.9}, "2": {...}} -- the whole fleet
        self.declare_parameter('roster_json', '{}')
        self.declare_parameter('tick_period', 8.0)       # s; must exceed one cell of driving
        self.declare_parameter('round_period', 0.15)     # s per negotiation round
        self.declare_parameter('rounds', 6)
        self.declare_parameter('arrive_tol', 0.25)       # m from cell centre = "arrived"

        g = self.get_parameter
        self.rid = int(g('robot_id').value)
        self.cs = float(g('cell_size').value)
        self.ox, self.oy = float(g('origin_x').value), float(g('origin_y').value)
        self.off_x, self.off_y = float(g('spawn_offset_x').value), float(g('spawn_offset_y').value)
        self.tick_period = float(g('tick_period').value)
        self.round_period = float(g('round_period').value)
        self.rounds = int(g('rounds').value)
        self.tol = float(g('arrive_tol').value)

        blocked = {tuple(c) for c in json.loads(g('blocked_cells_json').value)}
        self.nbrs = grid_graph(int(g('grid_w').value), int(g('grid_h').value), blocked)
        roster = json.loads(g('roster_json').value)
        me = roster.get(str(self.rid))
        if me is None:
            raise RuntimeError(f"robot_id {self.rid} missing from roster_json")

        self.agent = FleetAgent(self.rid, self.nbrs, float(g('priority').value),
                                tuple(me['cell']), None, rounds=self.rounds)
        for pid, info in roster.items():
            if int(pid) != self.rid:
                self.agent.seed_peer(int(pid), tuple(info['cell']), None,
                                     float(info.get('prio', 0.5)))

        self.world_x = self.world_y = None
        self.pending_goal = self.agent.goal          # applied only at a tick boundary
        self.goal_dirty = False
        self.pending_cell = None                     # cell we are currently driving to
        self.cur_tick = None
        self.next_round = 0
        self.skip_tick = False

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=200)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_cb, 10)
        self.create_subscription(String, '/fleet/pibt', self.peer_cb, qos)
        self.fleet_pub = self.create_publisher(String, '/fleet/pibt', qos)
        self.wp_pub = self.create_publisher(PoseStamped, 'nav_waypoint', 10)
        self.status_pub = self.create_publisher(String, 'pibt_status', 10)

        self.create_timer(0.02, self.spin)           # 50 Hz scheduler
        self.get_logger().info(
            f"pibt_fleet_node up: robot {self.rid}, start cell {self.agent.cell}, "
            f"tick {self.tick_period}s, {self.rounds} rounds x {self.round_period}s")

    # ---------------------------------------------------------------- helpers
    def to_cell(self, x, y):
        return (int(round((x - self.ox) / self.cs)), int(round((y - self.oy) / self.cs)))

    def centre(self, cell):
        return (self.ox + cell[0] * self.cs, self.oy + cell[1] * self.cs)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_json(self, msg):
        self.fleet_pub.publish(String(data=json.dumps(msg)))

    def send_waypoint(self, cell):
        x, y = self.centre(cell)
        wp = PoseStamped()
        wp.header.frame_id = 'map'
        wp.pose.position.x, wp.pose.position.y = float(x), float(y)
        wp.pose.orientation.w = 1.0
        self.wp_pub.publish(wp)

    # -------------------------------------------------------------- callbacks
    def odom_cb(self, msg):
        self.world_x = msg.pose.pose.position.x + self.off_x
        self.world_y = msg.pose.pose.position.y + self.off_y

    def goal_cb(self, msg):
        cell = self.to_cell(msg.pose.position.x, msg.pose.position.y)
        if cell not in self.nbrs:
            self.get_logger().warn(f"goal {cell} is not a free grid cell; ignored")
            return
        self.pending_goal, self.goal_dirty = cell, True
        self.get_logger().info(f"new goal cell {cell} (applied at next tick)")

    def peer_cb(self, msg):
        try:
            m = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if int(m.get('id', -1)) != self.rid:
            self.agent.receive(m)
            if len(self.agent._future) > 2000:          # bound the early-message buffer
                del self.agent._future[:1000]

    # ------------------------------------------------------------- scheduler
    def spin(self):
        if self.world_x is None:
            return
        now = self.now_sec()
        tick = int(now // self.tick_period)
        if tick != self.cur_tick:
            self.start_tick(tick)
        if self.skip_tick:
            return
        phase = now - tick * self.tick_period
        due = min(int(phase // self.round_period), self.rounds + 1)
        while self.next_round <= due:
            self.run_round(self.next_round)
            self.next_round += 1

    def start_tick(self, tick):
        self.cur_tick, self.next_round, self.skip_tick = tick, 0, False
        cell = self.to_cell(self.world_x, self.world_y)
        cx, cy = self.centre(cell)
        at_centre = math.hypot(self.world_x - cx, self.world_y - cy) <= self.tol
        if self.pending_cell is not None:
            if at_centre and cell == self.pending_cell:
                self.agent.target = self.pending_cell
                self.agent.apply()                       # move finished
                self.pending_cell = None
            elif at_centre and cell == self.agent.cell:
                self.pending_cell = None                 # never left: treat as stay
            else:
                # still between cells: sit this tick out. Peers keep us on their
                # map as {old cell, committed target}, so nobody drives into us.
                self.skip_tick = True
                self.get_logger().warn("still moving at tick boundary; skipping tick")
                return
        elif at_centre and cell != self.agent.cell:
            self.get_logger().warn(f"drifted to {cell}, expected {self.agent.cell}; resyncing")
            self.agent.cell = cell
        if self.goal_dirty:                              # goals change only between ticks
            self.agent.goal = self.pending_goal
            self.agent.docked = (self.agent.goal == self.agent.cell)
            self.goal_dirty = False

    def run_round(self, r):
        a = self.agent
        if r == 0:
            self.publish_json(a.begin_tick(self.cur_tick))
        elif r == 1:
            a.plan()
            self.publish_json(a.round_msg(1))
        elif r <= self.rounds:
            a.step_commit()
            self.publish_json(a.round_msg(r))
        else:                                            # r == rounds + 1: execute
            a.step_commit()
            target = a.finalize()
            if target != a.cell:
                self.pending_cell = target
            self.send_waypoint(target)                   # waypoint = hold if staying
            self.status_pub.publish(String(data=json.dumps({
                'id': self.rid, 'tick': self.cur_tick, 'cell': list(a.cell),
                'target': list(target), 'goal': list(a.goal) if a.goal else None,
                'docked': a.docked})))


def main(args=None):
    rclpy.init(args=args)
    node = PibtFleetNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
