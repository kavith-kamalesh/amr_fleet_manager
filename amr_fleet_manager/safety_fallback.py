import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class SafetySupervisor(Node):
    """Last stage before the motors: cmd_vel_nav -> [this node] -> cmd_vel.
    Always active (independent of comms). Fail-safe on stale command or stale LiDAR."""

    def __init__(self):
        super().__init__('safety_supervisor')
        self.declare_parameter('stop_distance', 0.35)
        self.declare_parameter('slow_distance', 0.80)
        self.declare_parameter('front_half_angle_deg', 45.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('scan_timeout_sec', 0.5)
        self.declare_parameter('require_scan', True)

        gp = lambda n: self.get_parameter(n).value
        self.stop_d = gp('stop_distance')
        self.slow_d = gp('slow_distance')
        self.half = math.radians(gp('front_half_angle_deg'))
        self.cmd_timeout = gp('cmd_timeout_sec')
        self.scan_timeout = gp('scan_timeout_sec')
        self.require_scan = gp('require_scan')

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Twist, 'cmd_vel_nav', self.cmd_cb, qos)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.state_pub = self.create_publisher(String, 'safety_state', qos)

        self.cmd = Twist()
        self.last_cmd_t = None
        self.front_min = None
        self.last_scan_t = None
        self._state = None

        self.create_timer(0.05, self.loop)
        self.get_logger().info("Safety supervisor active (always-on LiDAR gate).")

    def cmd_cb(self, msg: Twist):
        self.cmd = msg
        self.last_cmd_t = time.monotonic()

    def scan_cb(self, msg: LaserScan):
        m = math.inf
        a = msg.angle_min
        for r in msg.ranges:
            ang = math.atan2(math.sin(a), math.cos(a))
            if -self.half <= ang <= self.half and math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                if r < m:
                    m = r
            a += msg.angle_increment
        self.front_min = m
        self.last_scan_t = time.monotonic()

    def _set_state(self, s):
        if s != self._state:
            self._state = s
            self.state_pub.publish(String(data=s))
            if s != "OK":
                self.get_logger().warn(f"safety_state -> {s}")

    def loop(self):
        now = time.monotonic()
        out = Twist()

        if self.last_cmd_t is None or now - self.last_cmd_t > self.cmd_timeout:
            self._set_state("CMD_TIMEOUT")
        elif self.require_scan and (self.last_scan_t is None or now - self.last_scan_t > self.scan_timeout):
            self._set_state("FAILSAFE_NO_SCAN")
        else:
            out.linear.x = self.cmd.linear.x
            out.angular.z = self.cmd.angular.z
            state = "OK"
            if out.linear.x > 0.0 and self.front_min is not None:
                if self.front_min <= self.stop_d:
                    out.linear.x = 0.0
                    state = "STOP"
                elif self.front_min < self.slow_d:
                    out.linear.x *= (self.front_min - self.stop_d) / (self.slow_d - self.stop_d)
                    state = "SLOW"
            self._set_state(state)

        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
