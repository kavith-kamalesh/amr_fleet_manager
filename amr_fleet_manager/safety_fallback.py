import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class SafetyFallbackWatchdog(Node):
    def __init__(self):
        super().__init__('safety_fallback_watchdog')

        self.last_heartbeat = time.time()
        self.TIMEOUT_SEC = 0.5  # 500 ms heartbeat threshold

        self.create_subscription(Twist, 'peer_telemetry', self.heartbeat_cb, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_safe', 10)
        
        self.min_front_distance = 10.0
        self.timer = self.create_timer(0.05, self.watchdog_loop)
        self.get_logger().info("Safety Fallback Watchdog active (500ms timeout).")

    def heartbeat_cb(self, msg: Twist):
        self.last_heartbeat = time.time()

    def scan_cb(self, msg: LaserScan):
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if not (r != r or r == float('inf'))]
            if valid_ranges:
                self.min_front_distance = min(valid_ranges)

    def watchdog_loop(self):
        dt = time.time() - self.last_heartbeat
        
        if dt > self.TIMEOUT_SEC:
            self.get_logger().warn(
                f"[REACTIVE MODE] Heartbeat lost ({dt:.2f}s). LiDAR override active.",
                throttle_duration_sec=1.0
            )
            
            if self.min_front_distance < 0.45:
                emergency_stop = Twist()
                emergency_stop.linear.x = 0.0
                emergency_stop.angular.z = 0.4
                self.cmd_pub.publish(emergency_stop)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyFallbackWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
