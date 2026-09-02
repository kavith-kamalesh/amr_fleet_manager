import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SpatialMutex(Node):
    def __init__(self):
        super().__init__('spatial_mutex')
        
        # Configurable intersection zone (Adjust to match your warehouse layout)
        self.declare_parameter('intersection_x', 2.0)
        self.declare_parameter('intersection_y', 1.0)
        self.declare_parameter('mutex_radius', 1.2)
        
        self.int_x = self.get_parameter('intersection_x').value
        self.int_y = self.get_parameter('intersection_y').value
        self.radius = self.get_parameter('mutex_radius').value

        self.clearance_pub = self.create_publisher(String, 'mutex_clearance', 10)
        self.get_logger().info(f"Spatial Mutex active around intersection ({self.int_x}, {self.int_y}) with radius {self.radius}m")

def main(args=None):
    rclpy.init(args=args)
    node = SpatialMutex()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
