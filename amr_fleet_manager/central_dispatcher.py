import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from amr_fleet_manager.scenarios import SCENARIOS


class CentralFleetManager(Node):
    """Task INTAKE only. Traffic negotiation is decentralized (spatial_mutex + safety supervisor)."""

    def __init__(self):
        super().__init__('central_fleet_manager')
        # Goals are world coordinates (robot start + odom). See scenarios.py.
        self.declare_parameter('scenario', 'swap')
        scenario = self.get_parameter('scenario').value
        if scenario not in SCENARIOS:
            raise ValueError(f"unknown scenario {scenario!r}; choose from {sorted(SCENARIOS)}")
        self.goals = SCENARIOS[scenario]
        self.get_logger().info(f"scenario: {scenario}")
        self.pubs = {rid: self.create_publisher(PoseStamped, f'/{rid}/goal_pose', 10)
                     for rid in self.goals}
        self.pending = set(self.goals)
        self.timer = self.create_timer(1.0, self.dispatch_tasks)
        self.get_logger().info("Dispatcher up: sending each goal once its subscriber is discovered.")

    def dispatch_tasks(self):
        for rid in sorted(self.pending):
            pub = self.pubs[rid]
            if pub.get_subscription_count() == 0:
                continue
            x, y = self.goals[rid]
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.orientation.w = 1.0
            pub.publish(msg)
            self.pending.discard(rid)
            self.get_logger().info(f"Dispatched {rid} -> ({x}, {y})")
        if not self.pending:
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = CentralFleetManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
