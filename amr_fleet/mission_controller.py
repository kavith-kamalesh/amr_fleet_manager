import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        self.targets = {'robot1': {'x': 4.0, 'y': 2.0}, 'robot2': {'x': -3.0, 'y': 4.0}, 'robot3': {'x': 2.0, 'y': -3.0}}
        self.robots = ['robot1', 'robot2', 'robot3']
        self.current_poses = {r: {'x': 0.0, 'y': 0.0, 'yaw': 0.0} for r in self.robots}
        self.pubs = {r: self.create_publisher(Twist, f'/{r}/cmd_vel', 10) for r in self.robots}
        self.subs = {r: self.create_subscription(Odometry, f'/{r}/odom', lambda msg, id=r: self.odom_cb(msg, id), 10) for r in self.robots}
        self.timer = self.create_timer(0.1, self.control_loop)
    def odom_cb(self, msg, rid):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(o.w*o.z + o.x*o.y), 1.0 - 2.0*(o.y*o.y + o.z*o.z))
        self.current_poses[rid] = {'x': p.x, 'y': p.y, 'yaw': yaw}
    def control_loop(self):
        for r in self.robots:
            t, c = self.targets[r], self.current_poses[r]
            dx, dy = t['x'] - c['x'], t['y'] - c['y']
            dist = math.hypot(dx, dy)
            tw = Twist()
            if dist > 0.2:
                ad = math.atan2(dy, dx) - c['yaw']
                ad = math.atan2(math.sin(ad), math.cos(ad))
                if abs(ad) > 0.3: tw.angular.z = max(-0.8, min(0.8, 1.5 * ad))
                else:
                    tw.linear.x = max(0.0, min(0.3, 0.5 * dist))
                    tw.angular.z = max(-0.5, min(0.5, 1.0 * ad))
            self.pubs[r].publish(tw)

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
