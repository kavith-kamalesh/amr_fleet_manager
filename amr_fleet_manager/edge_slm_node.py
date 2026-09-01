"""
Edge SLM (Small Language Model) reasoning node.
Runs per-robot, in that robot's namespace, identically whether the robot
is a real Raspberry Pi or a Gazebo-simulated instance -- it only consumes
ROS2 topics already published by spatial_mutex, safety_fallback, and
waypoint_nav_node, never raw sensor/motor data. This keeps sim and real
deployment byte-for-byte identical.

Scope (deliberately limited): this node explains events in natural
language for a human operator dashboard. It is NOT in the real-time
control loop -- nothing here blocks or gates robot movement. If this
node is slow, crashes, or is entirely absent, the robot still navigates
and avoids collisions normally via spatial_mutex/safety_fallback.

Hardware note: on a Raspberry Pi Zero 2W, even a small (0.5B-1B param)
quantized model will take several seconds per explanation. This is
handled by (a) only triggering on notable events, never every tick,
and (b) running inference in a background thread so it can never stall
navigation or mutex logic.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
import json
import time
import threading

try:
    from llama_cpp import Llama
    LLAMA_AVAILABLE = True
except ImportError:
    LLAMA_AVAILABLE = False


class EdgeSLMNode(Node):
    def __init__(self):
        super().__init__('edge_slm_node')

        self.declare_parameter('robot_id', 1)
        self.declare_parameter('model_path', '')  # e.g. /home/pi/models/qwen2.5-0.5b-instruct-q4_k_m.gguf
        self.declare_parameter('min_seconds_between_calls', 4.0)  # throttle for slow hardware
        self.declare_parameter('long_wait_threshold_sec', 2.0)

        self.robot_id = self.get_parameter('robot_id').value
        model_path = self.get_parameter('model_path').value
        self.min_seconds_between_calls = self.get_parameter('min_seconds_between_calls').value
        self.long_wait_threshold = self.get_parameter('long_wait_threshold_sec').value

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(String, 'mutex_clearance', self.clearance_callback, qos)
        self.create_subscription(String, 'mission_status', self.mission_status_callback, qos)
        self.create_subscription(String, 'safety_event', self.safety_event_callback, qos)

        self.explanation_pub = self.create_publisher(String, 'status_explanation', qos)

        self.model = None
        if LLAMA_AVAILABLE and model_path:
            try:
                self.model = Llama(model_path=model_path, n_ctx=256, n_threads=2, verbose=False)
                self.get_logger().info(f"SLM loaded from {model_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load SLM model ({e}); falling back to templates")
        else:
            self.get_logger().warn("SLM not available (missing llama-cpp-python or model_path); "
                                    "using template-based explanations")

        self.last_call_time = 0.0
        self.busy = False
        self.wait_start_time = None

        self.get_logger().info(f"EdgeSLMNode up for robot_id={self.robot_id}")

    # ---------------- Event intake ----------------

    def clearance_callback(self, msg: String):
        state = msg.data
        now = time.time()

        if state == "WAIT":
            if self.wait_start_time is None:
                self.wait_start_time = now
            elif now - self.wait_start_time > self.long_wait_threshold:
                self.trigger_explanation({
                    'event': 'long_wait',
                    'robot_id': self.robot_id,
                    'wait_duration_sec': round(now - self.wait_start_time, 1),
                })
        elif state == "REROUTE_REQUESTED":
            self.trigger_explanation({
                'event': 'reroute',
                'robot_id': self.robot_id,
            })
            self.wait_start_time = None
        else:
            self.wait_start_time = None

    def mission_status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get('status') == 'ARRIVED':
                self.trigger_explanation({
                    'event': 'arrived',
                    'robot_id': self.robot_id,
                })
        except (json.JSONDecodeError, KeyError):
            pass

    def safety_event_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get('tier3_active'):
                self.trigger_explanation({
                    'event': 'safety_fallback',
                    'robot_id': self.robot_id,
                    'reason': data.get('reason', 'peer heartbeat lost'),
                })
        except (json.JSONDecodeError, KeyError):
            pass

    # ---------------- Explanation generation ----------------

    def trigger_explanation(self, event: dict):
        now = time.time()
        if self.busy or (now - self.last_call_time) < self.min_seconds_between_calls:
            # Throttled -- publish the cheap template version immediately
            # instead of dropping the event entirely, so slow hardware
            # still surfaces something to the operator.
            self.explanation_pub.publish(String(data=self.template_explanation(event)))
            return

        self.busy = True
        self.last_call_time = now
        thread = threading.Thread(target=self._generate_and_publish, args=(event,), daemon=True)
        thread.start()

    def _generate_and_publish(self, event: dict):
        try:
            if self.model is not None:
                text = self.generate_with_slm(event)
            else:
                text = self.template_explanation(event)
            self.explanation_pub.publish(String(data=text))
        finally:
            self.busy = False

    def generate_with_slm(self, event: dict) -> str:
        prompt = (
            "You are a warehouse robot reporting status to a human operator. "
            "Explain the following event in one short, plain sentence.\n"
            f"Event data: {json.dumps(event)}\n"
            "Explanation:"
        )
        try:
            output = self.model(
                prompt, max_tokens=40, temperature=0.3, stop=["\n"]
            )
            text = output['choices'][0]['text'].strip()
            return text if text else self.template_explanation(event)
        except Exception as e:
            self.get_logger().warn(f"SLM inference failed ({e}); using template")
            return self.template_explanation(event)

    def template_explanation(self, event: dict) -> str:
        etype = event.get('event')
        rid = event.get('robot_id')
        if etype == 'long_wait':
            return f"Robot {rid} has been waiting {event.get('wait_duration_sec')}s at an intersection for another robot to cross."
        if etype == 'reroute':
            return f"Robot {rid} is taking an alternate path to avoid a blocked intersection."
        if etype == 'arrived':
            return f"Robot {rid} has arrived at its destination."
        if etype == 'safety_fallback':
            return f"Robot {rid} lost contact with peer robots and is navigating cautiously using its own sensors ({event.get('reason')})."
        return f"Robot {rid} reported an event: {etype}"


def main(args=None):
    rclpy.init(args=args)
    node = EdgeSLMNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
