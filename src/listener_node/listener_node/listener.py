import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__("listener_node")
        self.subscription = self.create_subscription(
            String,
            "chatter",
            self.listener_callback,
            10)
        self.get_logger().info("Listener Node Started")

    def listener_callback(self, msg):
        self.get_logger().info(f"I heard: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Error occurred: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
