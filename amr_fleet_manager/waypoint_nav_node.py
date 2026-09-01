import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math

class WaypointNavNode(Node):
    def __init__(self):
        super().__init__('waypoint_nav_node')
        
        # Keep the Gazebo relative odometry patch!
        self.declare_parameter('spawn_offset_x', 0.0)
        self.declare_parameter('spawn_offset_y', 0.0)
        self.offset_x = self.get_parameter('spawn_offset_x').value
        self.offset_y = self.get_parameter('spawn_offset_y').value

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_x = None
        self.goal_y = None
        
        self.mutex_state = "CLEAR"

        # Subscriptions
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_cb, 10)
        self.create_subscription(String, 'mutex_clearance', self.mutex_cb, 10)
        
        # Publishers & Timers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f"Edge Nav Node initialized (Offset: {self.offset_x}, {self.offset_y})")

    def odom_cb(self, msg: Odometry):
        # Apply the spawn offset fix
        self.current_x = msg.pose.pose.position.x + self.offset_x
        self.current_y = msg.pose.pose.position.y + self.offset_y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_cb(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info(f"New FMS Goal Received: ({self.goal_x}, {self.goal_y})")

    def mutex_cb(self, msg: String):
        self.mutex_state = msg.data

    def control_loop(self):
        twist = Twist()
        
        # ==========================================
        # 1. THE EDGE AI EMERGENCY BRAKE
        # ==========================================
        if getattr(self, 'mutex_state', 'CLEAR') == "WAITING":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return  # Exit function completely. Do not drive.

        # ==========================================
        # 2. STANDARD LANE NAVIGATION
        # ==========================================
        if self.goal_x is None or self.goal_y is None:
            self.cmd_vel_pub.publish(twist)
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.2:
            self.goal_x = None
            self.get_logger().info("Target Bay Reached!")
        else:
            target_yaw = math.atan2(dy, dx)
            yaw_error = target_yaw - self.current_yaw
            
            # Normalize yaw error between -pi and pi
            while yaw_error > math.pi: yaw_error -= 2 * math.pi
            while yaw_error < -math.pi: yaw_error += 2 * math.pi

            # Steer towards goal
            twist.angular.z = max(min(yaw_error * 1.5, 1.0), -1.0)
            
            # Only drive forward if roughly pointing at the goal
            if abs(yaw_error) < 0.5:
                twist.linear.x = 0.2
            else:
                twist.linear.x = 0.0

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WaypointNavNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
