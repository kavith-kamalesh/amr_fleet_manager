import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class KinematicRobot(Node):
    """Minimal diff-drive integrator: cmd_vel in, odom out. No physics, no Gazebo."""

    def __init__(self):
        super().__init__('sim_kinematic_robot')
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.dt = 0.05
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_timer(self.dt, self.step)

    def cmd_cb(self, msg: Twist):
        self.v, self.w = msg.linear.x, msg.angular.z

    def step(self):
        self.yaw += self.w * self.dt
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id = 'odom'
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = math.sin(self.yaw / 2)
        o.pose.pose.orientation.w = math.cos(self.yaw / 2)
        o.twist.twist.linear.x = self.v
        o.twist.twist.angular.z = self.w
        self.odom_pub.publish(o)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicRobot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
