"""
Edge waypoint navigator. Runs per-robot, in that robot's own namespace
(e.g. /robot1). Subscribes to goal_pose from the central FMS, plans a
grid/graph A* path locally, and only advances along the path when the
local SpatialMutex node reports CLEAR for the current edge.

If the central FMS goes offline, this node simply keeps working toward
the last goal it received -- it never depends on the FMS being alive
to navigate or avoid collisions safely.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import heapq
import math

GRID_CELL = 1.0
GRID_SIZE = 10


def to_node(x, y):
    return (round(x / GRID_CELL), round(y / GRID_CELL))


def node_pos(n):
    return (n[0] * GRID_CELL, n[1] * GRID_CELL)


def neighbors(n):
    x, y = n
    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
            yield (nx, ny)


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(start, goal, blocked_edges=frozenset()):
    open_set = [(heuristic(start, goal), 0, start, [start])]
    visited = {}
    while open_set:
        f, g, current, path = heapq.heappop(open_set)
        if current == goal:
            return path
        if current in visited and visited[current] <= g:
            continue
        visited[current] = g
        for nxt in neighbors(current):
            edge = (current, nxt)
            if edge in blocked_edges or (nxt, current) in blocked_edges:
                continue
            ng = g + 1
            heapq.heappush(open_set, (ng + heuristic(nxt, goal), ng, nxt, path + [nxt]))
    return None


class WaypointNavNode(Node):
    def __init__(self):
        super().__init__('waypoint_nav_node')
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('max_speed', 0.8)
        # World-frame spawn offset: /odom is relative to each robot's own
        # start pose, NOT the Gazebo world origin. Every raw odom reading
        # must be shifted by this offset before it means anything in the
        # shared grid/graph the algorithm plans over. Set these to match
        # exactly the -x/-y values used in spawn_entity.py for this robot.
        self.declare_parameter('spawn_offset_x', 0.0)
        self.declare_parameter('spawn_offset_y', 0.0)
        self.robot_id = self.get_parameter('robot_id').value
        self.max_speed = self.get_parameter('max_speed').value
        self.spawn_offset_x = self.get_parameter('spawn_offset_x').value
        self.spawn_offset_y = self.get_parameter('spawn_offset_y').value

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

        # Namespaced automatically by ROS2 since this node runs inside
        # e.g. /robot1 -- so 'goal_pose' resolves to /robot1/goal_pose
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, qos)
        self.create_subscription(Odometry, 'odom', self.odom_callback, qos)
        self.create_subscription(String, 'mutex_clearance', self.clearance_callback, qos)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_nav', qos)
        self.status_pub = self.create_publisher(String, 'mission_status', qos)

        self.current_pos = (0.0, 0.0)
        self.path = None
        self.path_idx = 0
        self.clearance = "CLEAR"

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"WaypointNav up for robot_id={self.robot_id} (namespaced)")

    def odom_callback(self, msg: Odometry):
        # Convert this robot's LOCAL odom reading into WORLD/grid coordinates
        # by applying its spawn offset. Without this, every robot except the
        # one spawned at the origin reports the wrong position to the shared
        # grid/graph planner and the spatial mutex.
        self.current_pos = (
            msg.pose.pose.position.x + self.spawn_offset_x,
            msg.pose.pose.position.y + self.spawn_offset_y,
        )

    def goal_callback(self, msg: PoseStamped):
        start_node = to_node(*self.current_pos)
        goal_node = to_node(msg.pose.position.x, msg.pose.position.y)
        new_path = astar(start_node, goal_node)
        if new_path:
            self.path = new_path
            self.path_idx = 0
            self.get_logger().info(f"New goal received, path length {len(new_path)}")
        else:
            self.get_logger().warn("No path found to requested goal")

    def clearance_callback(self, msg: String):
        self.clearance = msg.data

    def control_loop(self):
        if self.path is None or self.path_idx >= len(self.path) - 1:
            return

        if self.clearance == "WAIT":
            self.cmd_pub.publish(Twist())  # full stop, no jitter
            return

        if self.clearance == "REROUTE_REQUESTED":
            current_node = to_node(*self.current_pos)
            goal_node = self.path[-1]
            blocked_edge = (self.path[self.path_idx], self.path[self.path_idx + 1])
            new_path = astar(current_node, goal_node, blocked_edges=frozenset({blocked_edge}))
            if new_path:
                self.path = new_path
                self.path_idx = 0
                self.get_logger().info("Rerouted around blocked edge")
            return

        target_node = self.path[self.path_idx + 1]
        target_xy = node_pos(target_node)
        dx = target_xy[0] - self.current_pos[0]
        dy = target_xy[1] - self.current_pos[1]
        dist = math.hypot(dx, dy)

        if dist < 0.15:
            self.path_idx += 1
            if self.path_idx >= len(self.path) - 1:
                self.status_pub.publish(String(data='{"status": "ARRIVED"}'))
            return

        speed = min(self.max_speed, dist)
        cmd = Twist()
        cmd.linear.x = (dx / dist) * speed
        cmd.linear.y = (dy / dist) * speed
        self.cmd_pub.publish(cmd)


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
