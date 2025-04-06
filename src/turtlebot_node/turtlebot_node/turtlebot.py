import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class TurtlebotNode(Node):
    def __init__(self):
        super().__init__("turtlebot_node")

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "cmd_vel",
            10
        )

        # Create subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10
        )

        self.chatter_sub = self.create_subscription(
            String,
            "chatter",
            self.chatter_callback,
            10
        )

        # Initialize robot state
        self.current_pose = None
        self.get_logger().info("Turtlebot Node Started")

        # Create a timer for movement updates
        self.create_timer(0.1, self.move_robot)

    def odom_callback(self, msg):
        """Update robot's pose from odometry"""
        self.current_pose = msg.pose.pose

    def chatter_callback(self, msg):
        """React to chatter messages"""
        self.get_logger().info(f"Turtlebot received: {msg.data}")

    def move_robot(self):
        """Simple movement pattern"""
        cmd = Twist()
        cmd.linear.x = 0.2  # Forward velocity
        cmd.angular.z = 0.1  # Angular velocity
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
