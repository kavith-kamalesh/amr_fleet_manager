"""
Central Fleet Management System (FMS) node.
Runs in the root namespace (/central_fms). Responsible ONLY for:
  - task allocation / mission planning (which robot goes where)
  - publishing high-level Goal poses per robot
  - global fleet monitoring (logging robot status)
It does NOT compute velocities, physics, or handle collision avoidance --
that is entirely the edge robots' responsibility via the Distributed
Spatial Mutex. If this node goes offline, edge robots continue operating
safely using their last received goal and local mutex logic.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
import time


class CentralDispatcher(Node):
    def __init__(self):
        super().__init__('central_dispatcher')

        self.declare_parameter('robot_names', ['robot1', 'robot2', 'robot3'])
        self.robot_names = self.get_parameter('robot_names').value

        # Simple hardcoded mission list for demo purposes: robot_name -> list of (x, y) goals
        self.declare_parameter('missions_json', '{}')
        missions_param = self.get_parameter('missions_json').value
        try:
            self.missions = json.loads(missions_param) if missions_param else {}
        except json.JSONDecodeError:
            self.missions = {}

        if not self.missions:
            # Fallback default missions if none provided via param
            self.missions = {
                'robot1': [[8.0, 8.0], [0.0, 0.0]],
                'robot2': [[0.0, 8.0], [8.0, 0.0]],
                'robot3': [[8.0, 0.0], [0.0, 8.0]],
            }

        self.mission_index = {name: 0 for name in self.robot_names}
        self.robot_status = {name: 'UNKNOWN' for name in self.robot_names}

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

        # Publish one goal topic per robot, namespaced explicitly
        self.goal_pubs = {}
        for name in self.robot_names:
            topic = f'/{name}/goal_pose'
            self.goal_pubs[name] = self.create_publisher(PoseStamped, topic, qos)

        # Subscribe to each robot's status/arrival reports
        for name in self.robot_names:
            topic = f'/{name}/mission_status'
            self.create_subscription(
                String, topic,
                lambda msg, n=name: self.status_callback(msg, n),
                qos
            )

        self.timer = self.create_timer(1.0, self.dispatch_loop)
        self.get_logger().info(f"Central FMS up. Managing robots: {self.robot_names}")

    def status_callback(self, msg: String, robot_name: str):
        try:
            data = json.loads(msg.data)
            status = data.get('status', 'UNKNOWN')
            self.robot_status[robot_name] = status
            if status == 'ARRIVED':
                self.mission_index[robot_name] += 1
                self.get_logger().info(f"[{robot_name}] arrived at goal. Advancing mission index.")
        except (json.JSONDecodeError, KeyError):
            self.get_logger().warn(f"Malformed status from {robot_name}, dropping")

    def dispatch_loop(self):
        for name in self.robot_names:
            missions = self.missions.get(name, [])
            idx = self.mission_index.get(name, 0)
            if idx >= len(missions):
                continue  # mission list exhausted for this robot

            goal_xy = missions[idx]
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = float(goal_xy[0])
            msg.pose.position.y = float(goal_xy[1])
            msg.pose.orientation.w = 1.0

            self.goal_pubs[name].publish(msg)

    def print_fleet_status(self):
        self.get_logger().info(f"Fleet status: {self.robot_status}")


def main(args=None):
    rclpy.init(args=args)
    node = CentralDispatcher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
