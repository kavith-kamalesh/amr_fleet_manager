import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class CentralFleetManager(Node):
    def __init__(self):
        super().__init__('central_fleet_manager')
        
        # Dedicated publishers for each namespace
        self.r1_goal_pub = self.create_publisher(PoseStamped, '/robot1/goal_pose', 10)
        self.r2_goal_pub = self.create_publisher(PoseStamped, '/robot2/goal_pose', 10)
        self.r3_goal_pub = self.create_publisher(PoseStamped, '/robot3/goal_pose', 10)
        
        # Dispatch tasks 5 seconds after launch to ensure Gazebo is fully loaded
        self.timer = self.create_timer(5.0, self.dispatch_tasks)
        self.get_logger().info("Central FMS Active: Waiting 5s to dispatch tasks...")

    def dispatch_tasks(self):
        # Create an intersection conflict for the SIH demo
        # Robot 1 goes to Bay A (needs to cross 0,0)
        goal1 = PoseStamped()
        goal1.pose.position.x = 2.0
        goal1.pose.position.y = 0.0
        self.r1_goal_pub.publish(goal1)
        
        # Robot 2 goes to Bay B (needs to cross 0,0 perpendicularly)
        goal2 = PoseStamped()
        goal2.pose.position.x = 0.0
        goal2.pose.position.y = 2.0
        self.r2_goal_pub.publish(goal2)
        
        self.get_logger().info("Demo Tasks Dispatched. Conflict generated at (0,0).")
        self.timer.cancel()  # Fire only once

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CentralFleetManager())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
