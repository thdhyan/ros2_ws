import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__("talker_node")
        self.publisher_ = self.create_publisher(String, "chatter", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
        self.get_logger().info("Talker Node Started")

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello World: {self.count}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Error occurred: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
