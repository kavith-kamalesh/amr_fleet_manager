import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import time

class WaypointNavNode(Node):
    def __init__(self):
        super().__init__('waypoint_nav_node')
        
        self.declare_parameter('target_x', 4.0)
        self.declare_parameter('target_y', 2.0)
        self.declare_parameter('bypass_threshold_sec', 3.0)
        self.declare_parameter('max_linear', 0.5)
        
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.bypass_threshold = self.get_parameter('bypass_threshold_sec').value
        self.max_linear = self.get_parameter('max_linear').value
        
        self.current_x = self.current_y = self.current_yaw = 0.0
        self.has_pose = False
        self.goal_active = True
        
        self.mutex_state = "CLEAR"
        self.wait_start_time = None
        self.detour_active = False

        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.create_subscription(String, 'mutex_clearance', self.mutex_callback, 10)
        self.publisher_cmd = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Time-Optimized Hybrid Waypoint Nav initialized.")

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        self.current_x, self.current_y = p.x, p.y
        self.has_pose = True

    def goal_callback(self, msg: PoseStamped):
        self.target_x, self.target_y = msg.pose.position.x, msg.pose.position.y
        self.goal_active = True
        self.detour_active = False
        self.wait_start_time = None
        self.get_logger().info(f"New primary goal: X={self.target_x:.2f}, Y={self.target_y:.2f}")

    def mutex_callback(self, msg: String):
        self.mutex_state = msg.data

    def trigger_dynamic_detour(self):
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        dist = math.hypot(dx, dy)
        
        if dist > 0.1:
            # Tighter 1.0m bypass to save time compared to previous 1.5m
            perp_x, perp_y = -dy / dist, dx / dist
            self.target_x += perp_x * 1.0
            self.target_y += perp_y * 1.0
            self.detour_active = True
            self.wait_start_time = None
            self.get_logger().warn(f"Rerouting to bypass target: X={self.target_x:.2f}, Y={self.target_y:.2f}")

    def control_loop(self):
        if not self.has_pose or not self.goal_active:
            return

        twist = Twist()
        
        # 1. Mutex Check
        if self.mutex_state == "WAIT" and not self.detour_active:
            if self.wait_start_time is None:
                self.wait_start_time = time.time()
            elif time.time() - self.wait_start_time > self.bypass_threshold:
                self.trigger_dynamic_detour()
            else:
                self.publisher_cmd.publish(twist)
                return
        else:
            self.wait_start_time = None

        # 2. Fluid Kinematic Drive (Continuous motion saves time)
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.hypot(dx, dy)
        
        if distance > 0.2:
            target_angle = math.atan2(dy, dx)
            angle_diff = (target_angle - self.current_yaw + math.pi) % (2 * math.pi) - math.pi
            
            # Fluid turning: Don't stop completely to turn unless the angle is extreme
            if abs(angle_diff) > 0.8:
                twist.angular.z = max(-1.0, min(1.0, 2.0 * angle_diff))
                twist.linear.x = 0.05 # Keep slight forward momentum
            else:
                twist.linear.x = max(0.1, min(self.max_linear, 0.8 * distance))
                twist.angular.z = max(-0.8, min(0.8, 1.5 * angle_diff))
        else:
            self.goal_active = False
            self.get_logger().info("Target waypoint reached.")

        self.publisher_cmd.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
