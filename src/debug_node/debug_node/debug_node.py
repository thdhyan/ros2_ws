import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.logging import LoggingSeverity


class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')

        # Print debug information
        self.get_logger().info("Debug node started")

        # Print ROS domain ID
        domain_id = os.getenv("ROS_DOMAIN_ID", "Not set")
        self.get_logger().info(f"ROS_DOMAIN_ID: {domain_id}")

        # Print ROS middleware
        rmw_impl = os.getenv("RMW_IMPLEMENTATION", "Default (rmw_fastrtps_cpp)")
        self.get_logger().info(f"RMW Implementation: {rmw_impl}")

        # Print ROS distro
        ros_distro = os.getenv("ROS_DISTRO", "Not set")
        self.get_logger().info(f"ROS Distro: {ros_distro}")

        # Set up parameters
        self.declare_parameter("topic1", "chatter1")
        self.declare_parameter("topic2", "chatter2")
        self.declare_parameter("log_level", "info")

        # Get parameters
        topic1 = self.get_parameter("topic1").value
        topic2 = self.get_parameter("topic2").value
        log_level = self.get_parameter("log_level").value

        # Set logging level
        self.set_logger_level(log_level)

        # Create subscribers
        self.subscription1 = self.create_subscription(
            String,
            topic1,
            lambda msg: self.topic_callback(msg, topic1),
            10
        )
        self.get_logger().info(f"Subscribed to topic: {topic1}")

        self.subscription2 = self.create_subscription(
            String,
            topic2,
            lambda msg: self.topic_callback(msg, topic2),
            10
        )
        self.get_logger().info(f"Subscribed to topic: {topic2}")

        self.get_logger().info(f"Debug node initialized with log level: {log_level}")

    def set_logger_level(self, log_level):
        if log_level == "debug":
            self.get_logger().set_level(LoggingSeverity.DEBUG)
        elif log_level == "info":
            self.get_logger().set_level(LoggingSeverity.INFO)
        elif log_level == "warn":
            self.get_logger().set_level(LoggingSeverity.WARN)
        elif log_level == "error":
            self.get_logger().set_level(LoggingSeverity.ERROR)
        else:
            self.get_logger().warn(f"Unknown log level '{log_level}', defaulting to 'info'")
            self.get_logger().set_level(LoggingSeverity.INFO)

    def topic_callback(self, msg, topic_name):
        self.get_logger().info(f"[{topic_name}] Received: '{msg.data}'")
        self.get_logger().debug(
            f"Message details - Topic: {topic_name}, Length: {len(msg.data)}, Address: {id(msg)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()