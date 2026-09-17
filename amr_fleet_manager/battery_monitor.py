"""
Battery telemetry publisher.

robot_common.RobotState.battery_level exists but nothing in the repo
writes to it. This node is the missing writer.

SIMULATION MODEL (not real hardware): drains proportional to linear
velocity command magnitude, recovers when health_status reports
CHARGING. This is a declared simulation model, not a claim of real
battery sensing -- say so if asked.
"""

import time
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('drain_rate_per_mps', 0.004)   # fraction/sec at max speed
        self.declare_parameter('idle_drain_rate', 0.0002)      # fraction/sec, always-on electronics
        self.declare_parameter('charge_rate', 0.02)             # fraction/sec while CHARGING
        self.declare_parameter('low_threshold', 0.25)
        self.declare_parameter('critical_threshold', 0.12)

        self.robot_id = self.get_parameter('robot_id').value
        self.drain_rate = self.get_parameter('drain_rate_per_mps').value
        self.idle_drain = self.get_parameter('idle_drain_rate').value
        self.charge_rate = self.get_parameter('charge_rate').value
        self.low_thresh = self.get_parameter('low_threshold').value
        self.crit_thresh = self.get_parameter('critical_threshold').value

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

        self.level = 1.0
        self.is_charging = False
        self.last_cmd_speed = 0.0
        self.last_tick = time.time()

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, qos)
        self.create_subscription(String, 'health_status', self.health_status_cb, qos)
        self.battery_pub = self.create_publisher(Float32, 'battery', qos)
        self.status_pub = self.create_publisher(String, 'battery_status', qos)

        self.create_timer(1.0, self.tick)
        self.get_logger().info(f"BatteryMonitor up | robot={self.robot_id}")

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_speed = (msg.linear.x ** 2 + msg.linear.y ** 2) ** 0.5

    def health_status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.is_charging = data.get('status') in ('CHARGING', 'SHIFT_CHANGE')
        except (json.JSONDecodeError, KeyError):
            pass

    def tick(self):
        now = time.time()
        dt = now - self.last_tick
        self.last_tick = now

        if self.is_charging:
            self.level = min(1.0, self.level + self.charge_rate * dt)
        else:
            drain = self.idle_drain + self.drain_rate * self.last_cmd_speed
            self.level = max(0.0, self.level - drain * dt)

        self.battery_pub.publish(Float32(data=self.level))

        state = "CHARGING" if self.is_charging else (
            "CRITICAL" if self.level < self.crit_thresh else
            "LOW" if self.level < self.low_thresh else "OK"
        )
        self.status_pub.publish(String(data=json.dumps({
            'robot_id': self.robot_id, 'level': round(self.level, 4), 'state': state
        })))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
