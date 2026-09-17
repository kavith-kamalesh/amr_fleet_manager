import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class TaskAuctioneer(Node):
    def __init__(self):
        super().__init__('task_auctioneer')
        self.pub = self.create_publisher(String, '/fleet/task_auction', 10)
        self.sub = self.create_subscription(String, '/fleet/task_auction', self.auction_cb, 10)
        
        # Sample warehouse manifest - add your SIH coordinates here
        self.tasks = [
            {"id": "Pallet_A", "x": 12.0, "y": 8.0},
            {"id": "Pallet_B", "x": 5.0, "y": 15.0},
            {"id": "Pallet_C", "x": 18.0, "y": 3.0}
        ]
        self.current_task = None
        self.bids = {}
        self.auction_end_time = 0
        self.state = "IDLE" 
        
        self.create_timer(0.5, self.loop)
        self.get_logger().info("Decentralized Task Auctioneer initialized (Contract-Net).")

    def auction_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            if payload.get("type") == "BID" and self.state == "AUCTIONING":
                if payload.get("task_id") == self.current_task["id"]:
                    self.bids[payload["robot_id"]] = payload["cost"]
        except Exception:
            pass

    def loop(self):
        if self.state == "IDLE":
            if self.tasks:
                self.current_task = self.tasks.pop(0)
                self.bids = {}
                self.state = "AUCTIONING"
                self.auction_end_time = time.time() + 2.0  # 2-second bidding window
                
                # Broadcast Call for Proposal (CFP)
                cfp = {"type": "CFP", "task_id": self.current_task["id"], "x": self.current_task["x"], "y": self.current_task["y"]}
                self.pub.publish(String(data=json.dumps(cfp)))
                self.get_logger().info(f"Broadcasted CFP for {self.current_task['id']} at ({self.current_task['x']}, {self.current_task['y']})")
            else:
                self.get_logger().info("All tasks auctioned successfully. Fleet is autonomous.")
                rclpy.shutdown()
                
        elif self.state == "AUCTIONING":
            if time.time() > self.auction_end_time:
                if self.bids:
                    # Contract-Net Protocol: Award to lowest cost
                    winner = min(self.bids, key=self.bids.get)
                    award = {"type": "AWARD", "task_id": self.current_task["id"], "winner_id": winner, "x": self.current_task["x"], "y": self.current_task["y"]}
                    self.pub.publish(String(data=json.dumps(award)))
                    self.get_logger().info(f"Awarded {self.current_task['id']} to {winner} (winning bid: {self.bids[winner]:.2f} cost)")
                else:
                    self.get_logger().warn(f"No bids received for {self.current_task['id']}. Re-queueing.")
                    self.tasks.append(self.current_task)
                
                self.state = "IDLE"
                time.sleep(1.0) 

def main(args=None):
    rclpy.init(args=args)
    node = TaskAuctioneer()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
