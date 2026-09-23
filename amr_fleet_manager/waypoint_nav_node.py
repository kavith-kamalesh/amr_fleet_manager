import json
import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from amr_fleet_manager import nav_graph
from amr_fleet_manager.robot_common import (
    MUTEX_CLEAR, MUTEX_WAIT, MUTEX_REROUTE, MUTEX_PARKED,
)


class WaypointNavNode(Node):
    def __init__(self):
        super().__init__('waypoint_nav_node')

        self.declare_parameter('spawn_offset_x', 0.0)
        self.declare_parameter('spawn_offset_y', 0.0)
        self.declare_parameter('robot_speed', 1.0)

        self.offset_x = self.get_parameter('spawn_offset_x').value
        self.offset_y = self.get_parameter('spawn_offset_y').value
        self.speed = self.get_parameter('robot_speed').value

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.path = None
        self.path_idx = 0
        self.blocked_edges = set()
        self.mutex_state = MUTEX_CLEAR
        self.edge_announced = False

        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_cb, 10)
        self.create_subscription(String, 'mutex_clearance', self.mutex_cb, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.planned_edge_pub = self.create_publisher(String, 'planned_edge', 10)
        self.task_complete_pub = self.create_publisher(String, 'task_complete', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            "waypoint_nav_node up: A* over the shared grid, motion gated on spatial_mutex clearance."
        )

    def odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x + self.offset_x
        self.current_y = msg.pose.pose.position.y + self.offset_y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_cb(self, msg: PoseStamped):
        start_node = nav_graph.node_of(self.current_x, self.current_y)
        goal_node = nav_graph.node_of(msg.pose.position.x, msg.pose.position.y)

        self.blocked_edges.clear()
        self.path = nav_graph.astar(start_node, goal_node, frozenset(self.blocked_edges))
        self.path_idx = 0
        self.edge_announced = False

        if self.path is None:
            self.get_logger().error(f"No A* path from {start_node} to {goal_node}")
        else:
            self.get_logger().info(f"New FMS goal ({msg.pose.position.x}, {msg.pose.position.y}) -> path {self.path}")

    def mutex_cb(self, msg: String):
        self.mutex_state = msg.data

    def current_edge(self):
        if self.path is None or self.path_idx >= len(self.path) - 1:
            return None
        return (self.path[self.path_idx], self.path[self.path_idx + 1])

    def announce_current_edge(self):
        edge = self.current_edge()
        if edge is None or self.edge_announced:
            return
        n1, n2 = edge
        t_start = time.time()
        t_end = t_start + nav_graph.CELL / self.speed
        payload = json.dumps({
            'nodes': [list(n1), list(n2)],
            't_start': t_start,
            't_end': t_end,
        })
        self.planned_edge_pub.publish(String(data=payload))
        self.edge_announced = True

    def control_loop(self):
        twist = Twist()

        if self.mutex_state == MUTEX_PARKED:
            self.cmd_vel_pub.publish(twist)
            return

        if self.path is None:
            self.cmd_vel_pub.publish(twist)
            return

        edge = self.current_edge()
        if edge is None:
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Goal reached.")
            self.task_complete_pub.publish(String(data="DONE"))
            self.path = None
            return

        self.announce_current_edge()

        if self.mutex_state == MUTEX_REROUTE:
            current_node = self.path[self.path_idx]
            goal_node = self.path[-1]
            self.blocked_edges.add(edge)
            new_path = nav_graph.astar(current_node, goal_node, frozenset(self.blocked_edges))
            if new_path:
                self.get_logger().warn(f"Rerouting around blocked edge {edge} -> {new_path}")
                self.path = new_path
                self.path_idx = 0
                self.edge_announced = False
            else:
                self.get_logger().warn(f"No alternate route around {edge}; continuing to wait.")
            self.cmd_vel_pub.publish(twist)
            return

        if self.mutex_state == MUTEX_WAIT:
            self.cmd_vel_pub.publish(twist)
            return

        n2 = edge[1]
        target = nav_graph.node_pos(n2)
        dx = target[0] - self.current_x
        dy = target[1] - self.current_y
        distance = math.hypot(dx, dy)

        if distance < 0.2:
            self.path_idx += 1
            self.edge_announced = False
            self.cmd_vel_pub.publish(twist)
            return

        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self.current_yaw
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi

        twist.angular.z = max(min(yaw_error * 1.5, 1.0), -1.0)
        twist.linear.x = self.speed if abs(yaw_error) < 0.5 else 0.0

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WaypointNavNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
