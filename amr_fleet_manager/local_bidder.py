import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import json
import math

class LocalBidder(Node):
    def __init__(self):
        super().__init__('local_bidder')
        self.declare_parameter('robot_id', 'robot1')
        self.robot_id = self.get_parameter('robot_id').value
        
        self.current_x = self.current_y = 0.0
        self.has_pose = False
        
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.sub_auction = self.create_subscription(String, '/fleet/task_auction', self.auction_cb, 10)
        self.pub_auction = self.create_publisher(String, '/fleet/task_auction', 10)
        self.pub_goal = self.create_publisher(PoseStamped, 'goal_pose', 10) # Feeds your existing waypoint node
        
        self.get_logger().info(f"{self.robot_id} bidder node active. Listening for P2P CFPs.")

    def odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.has_pose = True

    def auction_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
            msg_type = payload.get("type")
            
            if msg_type == "CFP" and self.has_pose:
                # Edge AI metric: Calculate hardware cost locally
                task_x, task_y = float(payload.get("x")), float(payload.get("y"))
                cost = math.hypot(task_x - self.current_x, task_y - self.current_y)
                
                bid = {
                    "type": "BID",
                    "task_id": payload.get("task_id"),
                    "robot_id": self.robot_id,
                    "cost": cost
                }
                self.pub_auction.publish(String(data=json.dumps(bid)))
                self.get_logger().info(f"Submitted bid of {cost:.2f} for {payload.get('task_id')}")
                
            elif msg_type == "AWARD":
                if payload.get("winner_id") == self.robot_id:
                    self.get_logger().info(f"WON task {payload.get('task_id')}! Dispatching to local navigator.")
                    goal = PoseStamped()
                    goal.header.frame_id = "map"
                    goal.header.stamp = self.get_clock().now().to_msg()
                    goal.pose.position.x = float(payload.get("x"))
                    goal.pose.position.y = float(payload.get("y"))
                    self.pub_goal.publish(goal)
                    
        except json.JSONDecodeError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = LocalBidder()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
