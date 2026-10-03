import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

HALT_STATES = ("WAIT", "WAITING", "PARKED")
DETOUR_OFFSET = 1.5


class WaypointNavNode(Node):
    def __init__(self):
        super().__init__('waypoint_nav_node')
        self.declare_parameter('spawn_offset_x', 0.0)
        self.declare_parameter('spawn_offset_y', 0.0)
        self.offset_x = self.get_parameter('spawn_offset_x').value
        self.offset_y = self.get_parameter('spawn_offset_y').value

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_x = None
        self.goal_y = None
        self.via = []            # detour waypoints, visited before the real goal
        self.mutex_state = "CLEAR"
        self.rerouted = False

        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_cb, 10)
        self.create_subscription(String, 'mutex_clearance', self.mutex_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Waypoint nav node up (halt-on-WAIT + detour reroute).")

    def odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x + self.offset_x
        self.current_y = msg.pose.pose.position.y + self.offset_y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y * q.y + q.z * q.z))

    def goal_cb(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.via = []
        self.rerouted = False
        self.get_logger().info(f"New goal: ({self.goal_x}, {self.goal_y})")

    def mutex_cb(self, msg: String):
        self.mutex_state = msg.data
        if (msg.data == "REROUTE_REQUESTED" and not self.rerouted
                and self.goal_x is not None):
            dx, dy = self.goal_x - self.current_x, self.goal_y - self.current_y
            d = math.hypot(dx, dy) or 1.0
            ux, uy = dx / d, dy / d
            # midpoint of the path, pushed sideways: real goal is preserved
            vx = self.current_x + dx / 2 - uy * DETOUR_OFFSET
            vy = self.current_y + dy / 2 + ux * DETOUR_OFFSET
            self.via.append((vx, vy))
            self.rerouted = True
            self.mutex_state = "CLEAR"
            self.get_logger().warn(f"Reroute via ({vx:.2f}, {vy:.2f})")

    def control_loop(self):
        twist = Twist()
        if self.goal_x is None:
            self.cmd_vel_pub.publish(twist)
            return
        if self.mutex_state in HALT_STATES:
            self.cmd_vel_pub.publish(twist)   # hold position
            return

        tx, ty = self.via[0] if self.via else (self.goal_x, self.goal_y)
        dx, dy = tx - self.current_x, ty - self.current_y
        if math.hypot(dx, dy) < (0.3 if self.via else 0.2):
            if self.via:
                self.via.pop(0)
            else:
                self.goal_x = self.goal_y = None
                self.rerouted = False
                self.get_logger().info("Target reached.")
        else:
            err = math.atan2(dy, dx) - self.current_yaw
            while err > math.pi: err -= 2 * math.pi
            while err < -math.pi: err += 2 * math.pi
            twist.angular.z = max(min(err * 1.5, 1.0), -1.0)
            twist.linear.x = 0.2 if abs(err) < 0.5 else 0.0
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WaypointNavNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
