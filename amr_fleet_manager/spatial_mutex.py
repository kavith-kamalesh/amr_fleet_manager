import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

class DistributedSpatialMutex(Node):
    def __init__(self):
        super().__init__('spatial_mutex')
        
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('priority', 1) # Lower number = higher priority
        
        self.robot_id = self.get_parameter('robot_id').value
        self.priority = self.get_parameter('priority').value
        
        # Local state
        self.current_x = 0.0
        self.current_y = 0.0
        self.intersection_lock = False
        
        # Subscriptions
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(String, '/peer_telemetry', self.peer_cb, 10)
        
        # Publishers
        self.state_pub = self.create_publisher(String, 'mutex_clearance', 10)
        self.broadcast_pub = self.create_publisher(String, '/peer_telemetry', 10)
        
        self.timer = self.create_timer(0.1, self.mutex_loop)
        self.get_logger().info(f"Edge Mutex active for Robot {self.robot_id} (Priority {self.priority})")

    def odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def peer_cb(self, msg: String):
        # Listen to ESP-NOW simulated peer network for higher priority locks
        if "LOCK" in msg.data:
            sender_priority = int(msg.data.split('_')[1])
            if sender_priority < self.priority and self.is_near_intersection():
                self.intersection_lock = True
        elif "UNLOCK" in msg.data:
            self.intersection_lock = False

    def is_near_intersection(self):
        # Simplified for SIH demo: The primary intersection is at (0,0)
        dist_to_center = math.sqrt(self.current_x**2 + self.current_y**2)
        return dist_to_center < 1.0

    def mutex_loop(self):
        state = String()
        if self.is_near_intersection():
            if self.intersection_lock:
                state.data = "WAITING"
            else:
                state.data = "CROSSING"
                # Broadcast lock claim to swarm
                lock_msg = String()
                lock_msg.data = f"LOCK_{self.priority}_{self.robot_id}"
                self.broadcast_pub.publish(lock_msg)
        else:
            state.data = "CLEAR"
            if not self.intersection_lock:
                # Release lock if we just left the intersection
                unlock_msg = String()
                unlock_msg.data = f"UNLOCK_{self.priority}_{self.robot_id}"
                self.broadcast_pub.publish(unlock_msg)
                
        self.state_pub.publish(state)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DistributedSpatialMutex())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
