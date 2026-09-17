import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from llama_cpp import Llama

class EdgeSLMNode(Node):
    def __init__(self):
        super().__init__('edge_slm_node')
        self.get_logger().info("Loading Qwen2.5-0.5B (Thread Pinned)...")
        self.llm = Llama(model_path="models/qwen2.5-0.5b-instruct-q4_k_m.gguf", n_threads=2)
        
        self.subscription = self.create_subscription(String, '/robot1/state', self.state_cb, 10)
        self.explanation_pub = self.create_publisher(String, '/robot1/state_explanation', 10)
        self.busy = False 

    def state_cb(self, msg):
        if self.busy:
            return  # Drop request if inference is already running
        
        self.busy = True
        threading.Thread(target=self.generate_explanation, args=(msg.data,), daemon=True).start()

    def generate_explanation(self, state_text):
        prompt = f"The robot state is {state_text}. Explain this in one short sentence."
        response = self.llm(prompt, max_tokens=30, echo=False)
        
        out_msg = String()
        out_msg.data = response['choices'][0]['text'].strip()
        self.explanation_pub.publish(out_msg)
        self.busy = False

def main(args=None):
    rclpy.init(args=args)
    node = EdgeSLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
