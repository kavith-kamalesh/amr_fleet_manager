import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import String
import json
import time
import threading

try:
    from cv_bridge import CvBridge
    import cv2
    VISION_LIBS_AVAILABLE = True
except ImportError:
    VISION_LIBS_AVAILABLE = False

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False


class LidarBlindspotFallback(Node):
    def __init__(self):
        super().__init__('lidar_blindspot_fallback')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('ambiguous_range_min', 0.0)   # LiDAR returns exactly 0 or NaN -> suspicious
        self.declare_parameter('min_seconds_between_checks', 5.0)

        self.robot_id = self.get_parameter('robot_id').value
        model_path = self.get_parameter('model_path').value
        self.min_interval = self.get_parameter('min_seconds_between_checks').value

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=5)

        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)
        self.create_subscription(Image, 'camera/image_raw', self.image_callback, qos)
        self.result_pub = self.create_publisher(String, 'blindspot_detections', qos)

        self.bridge = CvBridge() if VISION_LIBS_AVAILABLE else None
        self.model = None
        if ULTRALYTICS_AVAILABLE:
            try:
                self.model = YOLO(model_path)
            except Exception as e:
                self.get_logger().warn(f"Could not load YOLO model ({e})")

        self.latest_frame = None
        self.last_check_time = 0.0
        self.busy = False

        if not VISION_LIBS_AVAILABLE:
            self.get_logger().warn("cv_bridge/opencv not available -- blind-spot fallback is inert")

        self.get_logger().info(f"LidarBlindspotFallback up for robot_id={self.robot_id} (triggered-only)")

    def image_callback(self, msg: Image):
        # Always keep the latest frame buffered, cheaply, so it's ready
        # the instant LiDAR flags something ambiguous -- but never
        # process it here.
        self.latest_frame = msg

    def scan_callback(self, msg: LaserScan):
        if not self._has_ambiguous_reading(msg):
            return

        now = time.time()
        if self.busy or (now - self.last_check_time) < self.min_interval:
            return  # throttled -- this is the "only log, don't process" rule

        if self.latest_frame is None or not VISION_LIBS_AVAILABLE:
            return

        self.last_check_time = now
        self.busy = True
        thread = threading.Thread(target=self._check_frame, args=(self.latest_frame,), daemon=True)
        thread.start()

    def _has_ambiguous_reading(self, scan: LaserScan) -> bool:
        
        if not scan.ranges:
            return False
        suspicious = sum(
            1 for r in scan.ranges
            if r != r or r <= 0.01 or r >= scan.range_max - 0.01
        )
        return (suspicious / len(scan.ranges)) > 0.15  # >15% of the scan looks wrong

    def _check_frame(self, image_msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            detections = []
            if self.model is not None:
                results = self.model(cv_image, verbose=False)
                for r in results:
                    for box in r.boxes:
                        conf = float(box.conf[0])
                        if conf < 0.4:
                            continue
                        cls_id = int(box.cls[0])
                        detections.append({
                            'label': self.model.names.get(cls_id, str(cls_id)),
                            'confidence': round(conf, 2),
                        })
            payload = json.dumps({
                'robot_id': self.robot_id,
                'trigger': 'lidar_ambiguous_reading',
                'timestamp': time.time(),
                'detections': detections,
            })
            self.result_pub.publish(String(data=payload))
        except Exception as e:
            self.get_logger().warn(f"Blind-spot check failed: {e}")
        finally:
            self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = LidarBlindspotFallback()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
