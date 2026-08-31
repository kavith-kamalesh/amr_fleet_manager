import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rvo2

class ORCANavigationNode(Node):
    def __init__(self):
        super().__init__('orca_nav_node')
        
        # Declare parameters
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('max_neighbors', 5)
        self.declare_parameter('neighbor_dist', 2.0)
        self.declare_parameter('time_horizon', 1.5)
        self.declare_parameter('radius', 0.35)
        self.declare_parameter('max_speed', 1.0)
        
        self.robot_id = self.get_parameter('robot_id').value
        max_neighbors = self.get_parameter('max_neighbors').value
        neighbor_dist = self.get_parameter('neighbor_dist').value
        time_horizon = self.get_parameter('time_horizon').value
        radius = self.get_parameter('radius').value
        max_speed = self.get_parameter('max_speed').value

        # QoS configuration for low-latency P2P state exchange
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 1. Preferred velocity from Nav2 A* planner
        self.pref_sub = self.create_subscription(
            Twist, 'cmd_vel_nav', self.pref_vel_callback, qos
        )

        # 2. Local self-odometry
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.self_odom_callback, qos
        )

        # 3. Safe velocity publisher to motors / Gazebo
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize RVO2 / ORCA simulator instance
        self.sim = rvo2.PyRVOSimulator(0.05, neighbor_dist, max_neighbors, time_horizon, 1.0, radius, max_speed)
        self.self_agent = self.sim.addAgent((0.0, 0.0))
        
        self.preferred_vel = (0.0, 0.0)

        # Run loop at 20 Hz (50 ms)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"ORCA Node initialized for robot_{self.robot_id}")

    def pref_vel_callback(self, msg: Twist):
        self.preferred_vel = (msg.linear.x, msg.linear.y)

    def self_odom_callback(self, msg: Odometry):
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.sim.setAgentPosition(self.self_agent, pos)
        self.sim.setAgentVelocity(self.self_agent, vel)

    def control_loop(self):
        self.sim.setAgentPrefVelocity(self.self_agent, self.preferred_vel)
        self.sim.doStep()
        safe_vel = self.sim.getAgentVelocity(self.self_agent)
        
        cmd = Twist()
        cmd.linear.x = safe_vel[0]
        cmd.linear.y = safe_vel[1]
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ORCANavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
