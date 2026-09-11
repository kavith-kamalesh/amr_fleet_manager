import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json
import time


class VerifyLogNotProcess(Node):
    def __init__(self):
        super().__init__('verify_log_not_process')
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

        self.intent_pub = self.create_publisher(String, '/fleet/spatial_intent', qos)
        self.clearance_msgs_seen = []
        self.create_subscription(String, 'mutex_clearance', self.clearance_cb, qos)

        self.timer = self.create_timer(1.0, self.send_fake_intent)
        self.count = 0

    def clearance_cb(self, msg: String):
        self.clearance_msgs_seen.append((time.time(), msg.data))
        self.get_logger().info(f"mutex_clearance fired: {msg.data} "
                                f"<- if the robot is CHARGING, this should NOT happen")

    def send_fake_intent(self):
        self.count += 1
        payload = json.dumps({
            'id': 999,  # fake peer, distinct from any real robot_id
            'nodes': [[0, 0], [0, 1]],
            'window': [time.time(), time.time() + 1.0],
            'priority': 0.9,
        })
        self.intent_pub.publish(String(data=payload))
        self.get_logger().info(f"Sent fake peer intent #{self.count} "
                                f"-- check the target robot's background_position_log grows")


def main(args=None):
    rclpy.init(args=args)
    node = VerifyLogNotProcess()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
