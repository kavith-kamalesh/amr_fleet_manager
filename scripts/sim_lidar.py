#!/usr/bin/env python3
"""Geometry-driven fake LiDAR for the ROS smoke test (no physics engine needed).

Publishes /robotN/scan for every robot. Each scan is built from the OTHER robots' positions,
modelled as discs of radius BODY_R, so safety_fallback's e-stop fires from real proximity
instead of from a script. Positions are world-frame (odom + spawn offset), like the launch files.

Limits: it senses robots only (no racks or walls), and ranges are exact, with no noise.
"""
import math

OFFSETS = {"robot1": (0.0, 0.0), "robot2": (4.0, 0.0), "robot3": (0.0, 4.0)}
BODY_R = 0.3
N_RAYS = 360
RANGE_MAX = 10.0
ANGLE_MIN = -math.pi
INC = 2 * math.pi / N_RAYS


def scan_ranges(self_xy, self_yaw, others_xy, body_r=BODY_R, n=N_RAYS,
                angle_min=ANGLE_MIN, range_max=RANGE_MAX):
    """Range per ray (angle = angle_min + k*increment, measured from the robot's heading)."""
    inc = 2 * math.pi / n
    ranges = [range_max] * n
    for ox, oy in others_xy:
        dx, dy = ox - self_xy[0], oy - self_xy[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            continue
        surface = max(dist - body_r, 0.02)
        if surface >= range_max:
            continue
        bearing = math.atan2(dy, dx) - self_yaw
        half = math.asin(min(1.0, body_r / dist))
        for k in range(n):
            a = angle_min + k * inc
            diff = math.atan2(math.sin(a - bearing), math.cos(a - bearing))
            if abs(diff) <= half:
                ranges[k] = min(ranges[k], surface)
    return ranges


def run():
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node("sim_lidar")
    state = {}

    def make_cb(name):
        ox, oy = OFFSETS[name]

        def cb(msg):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            state[name] = (p.x + ox, p.y + oy, 2.0 * math.atan2(q.z, q.w))
        return cb

    pubs = {}
    for name in OFFSETS:
        node.create_subscription(Odometry, f"/{name}/odom", make_cb(name), 10)
        pubs[name] = node.create_publisher(LaserScan, f"/{name}/scan", 10)

    def tick():
        for name, (x, y, yaw) in list(state.items()):
            others = [(sx, sy) for n, (sx, sy, _) in state.items() if n != name]
            msg = LaserScan()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = f"{name}/base_scan"
            msg.angle_min = ANGLE_MIN
            msg.angle_max = ANGLE_MIN + (N_RAYS - 1) * INC
            msg.angle_increment = INC
            msg.range_min = 0.05
            msg.range_max = RANGE_MAX
            msg.ranges = scan_ranges((x, y), yaw, others)
            pubs[name].publish(msg)

    node.create_timer(0.1, tick)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    run()
