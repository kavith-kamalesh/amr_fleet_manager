import math, json, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class StopAndWaitNode(Node):
    def __init__(self):
        super().__init__('stop_and_wait_node')
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('halt_radius', 1.5)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.0)
        
        self.robot_id = self.get_parameter('robot_id').value
        self.halt_radius = self.get_parameter('halt_radius').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.current_x = self.current_y = self.current_yaw = 0.0
        self.goal_x = self.goal_y = None
        self.peer_positions = {}
        
        self.create_subscription(Odometry, 'odom', self.odom_cb, qos)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_cb, qos)
        self.create_subscription(String, '/fleet/raw_positions', self.peer_cb, qos)
        
        self.pos_pub = self.create_publisher(String, '/fleet/raw_positions', qos)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'baseline_status', 10)
        
        self.create_timer(0.1, self.broadcast_position)
        self.create_timer(0.1, self.control_loop)

    def odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def goal_cb(self, msg: PoseStamped):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

    def peer_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
            if d['id'] != self.robot_id:
                self.peer_positions[d['id']] = (d['x'], d['y'])
        except: pass

    def broadcast_position(self):
        self.pos_pub.publish(String(data=json.dumps({'id': self.robot_id, 'x': self.current_x, 'y': self.current_y})))

    def any_peer_too_close(self):
        for pid, (px, py) in self.peer_positions.items():
            if math.dist((self.current_x, self.current_y), (px, py)) < self.halt_radius: return pid
        return None

    def control_loop(self):
        twist = Twist()
        if self.goal_x is None:
            self.cmd_pub.publish(twist)
            return
            
        dx, dy = self.goal_x - self.current_x, self.goal_y - self.current_y
        if math.hypot(dx, dy) < 0.2:
            self.goal_x = None
            self.status_pub.publish(String(data="ARRIVED"))
            self.cmd_pub.publish(twist)
            return
            
        blocker = self.any_peer_too_close()
        if blocker is not None:
            self.cmd_pub.publish(Twist())
            self.status_pub.publish(String(data=f"HALTED_near_{blocker}"))
            return
            
        target_yaw = math.atan2(dy, dx)
        yaw_error = (target_yaw - self.current_yaw + math.pi) % (2 * math.pi) - math.pi
        twist.angular.z = max(min(yaw_error * 1.5, self.max_angular), -self.max_angular)
        twist.linear.x = self.max_linear if abs(yaw_error) < 0.5 else 0.0
        
        self.cmd_pub.publish(twist)
        self.status_pub.publish(String(data="MOVING"))

def main(args=None):
    rclpy.init(args=args)
    node = StopAndWaitNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
