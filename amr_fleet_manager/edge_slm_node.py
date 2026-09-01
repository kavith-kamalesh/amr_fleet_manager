import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class EdgeSLMNode(Node):
    def __init__(self):
        super().__init__('edge_slm_node')
        
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('model_path', '')
        
        self.robot_id = self.get_parameter('robot_id').value
        self.model_path = self.get_parameter('model_path').value
        
        self.llm = None
        self.setup_ai()

        # Listen to the robot's internal state
        self.create_subscription(String, 'mutex_clearance', self.state_cb, 10)
        
        # Publish the plain English explanation
        self.explanation_pub = self.create_publisher(String, 'status_explanation', 10)
        
        self.last_state = ""
        self.get_logger().info(f"Edge AI (SLM) initialized for Robot {self.robot_id}")

    def setup_ai(self):
        if self.model_path:
            try:
                from llama_cpp import Llama
                self.llm = Llama(model_path=self.model_path, n_ctx=256, verbose=False)
                self.get_logger().info("Hardware SLM loaded successfully.")
            except ImportError:
                self.get_logger().warn("llama-cpp-python not found. Using template fallback.")
            except Exception as e:
                self.get_logger().warn(f"Model load failed: {e}. Using template fallback.")
        else:
            self.get_logger().info("No model path provided. Running in Explainable AI fallback mode.")

    def state_cb(self, msg: String):
        current_state = msg.data
        if current_state != self.last_state:
            explanation = self.generate_explanation(current_state)
            
            out_msg = String()
            out_msg.data = explanation
            self.explanation_pub.publish(out_msg)
            
            self.get_logger().info(f"[XAI] {explanation}")
            self.last_state = current_state

    def generate_explanation(self, state):
        prompt = f"You are an industrial robot. Your current intersection state is: {state}. Explain what you are doing in one short sentence."
        
        # If true Edge AI is loaded, generate inference
        if self.llm:
            try:
                output = self.llm(prompt, max_tokens=30, stop=["\n", "Robot:"])
                return output['choices'][0]['text'].strip()
            except Exception as e:
                self.get_logger().error(f"Inference error: {e}")
        
        # Explainable AI Template Fallback (For instant hackathon demos)
        if state == "WAITING":
            return "I am halting at the intersection because another unit has right-of-way."
        elif state == "CROSSING":
            return "I have acquired the spatial lock and am now crossing the intersection."
        elif state == "CLEAR":
            return "The path is clear. I am proceeding along my designated route."
        
        return f"Transitioning to state: {state}"

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(EdgeSLMNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
