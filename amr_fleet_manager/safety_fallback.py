import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# Matches the MIN_SAFE_DIST margin used in benchmark_stop_and_wait_vs_hybrid.py's
# geometric safety layer -- that benchmark demonstrated this class of check is
# structurally necessary: the topological mutex only governs who may ENTER a
# contested edge, it says nothing about whether the space directly ahead is
# physically clear right now.
SAFE_STOP_DISTANCE = 0.45   # meters
CLEAR_HYSTERESIS_DISTANCE = 0.55  # must clear past this before releasing, to avoid STOP/CLEAR flapping right at the threshold
FRONT_HALF_ANGLE_RAD = math.radians(30)  # only the forward +/-30 degree sector triggers a stop


def front_sector_min_range(angle_min, angle_increment, ranges, half_angle_rad=FRONT_HALF_ANGLE_RAD):
    """Pure function, independent of ROS message types, so it's testable
    without rclpy. angle_min/angle_increment come straight off the
    LaserScan message -- ranges[i] corresponds to angle_min + i*increment,
    which is the actual indexing contract, not an assumption that index 0
    means straight ahead (it usually doesn't)."""
    min_range = float('inf')
    angle = angle_min
    for r in ranges:
        if abs(angle) <= half_angle_rad:
            if not (r != r) and r > 0.01:  # reject NaN and zero/garbage readings
                min_range = min(min_range, r)
        angle += angle_increment
    return min_range


class SafetyFallbackWatchdog(Node):
    """
    Pure geometric last-resort safety layer, independent of spatial_mutex
    and independent of any peer/negotiation state. Watches the LiDAR
    front sector and publishes a STOP/CLEAR signal that waypoint_nav_node
    treats as the highest-priority override -- above even a CLEAR mutex
    clearance -- because "you have right of way" is a topological/temporal
    statement, not a promise the space ahead is actually empty right now.

    Previous version of this file waited on a 'peer_telemetry' heartbeat
    that nothing in the system ever published, so it was permanently
    stuck in "REACTIVE MODE" and published to 'cmd_vel_safe', a topic
    nothing subscribed to -- it could never have actually stopped a
    robot. This version has no heartbeat dependency at all: it is a
    single-purpose LiDAR proximity check, and its output topic
    ('emergency_stop') is consumed directly by waypoint_nav_node.
    """

    def __init__(self):
        super().__init__('safety_fallback_watchdog')

        self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        self.stop_pub = self.create_publisher(String, 'emergency_stop', 10)

        self.stopped = False
        self.get_logger().info(
            f"Safety Fallback Watchdog active: LiDAR front-sector "
            f"(+/-{math.degrees(FRONT_HALF_ANGLE_RAD):.0f} deg) emergency stop "
            f"below {SAFE_STOP_DISTANCE}m."
        )

    def scan_cb(self, msg: LaserScan):
        if not msg.ranges:
            return

        front_min = front_sector_min_range(msg.angle_min, msg.angle_increment, msg.ranges)

        if not self.stopped and front_min < SAFE_STOP_DISTANCE:
            self.stopped = True
            self.get_logger().warn(
                f"EMERGENCY STOP: obstacle at {front_min:.2f}m in front sector "
                f"(threshold {SAFE_STOP_DISTANCE}m)."
            )
        elif self.stopped and front_min > CLEAR_HYSTERESIS_DISTANCE:
            self.stopped = False
            self.get_logger().info(
                f"Front sector clear ({front_min:.2f}m) -- releasing emergency stop."
            )

        self.stop_pub.publish(String(data="STOP" if self.stopped else "CLEAR"))


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
