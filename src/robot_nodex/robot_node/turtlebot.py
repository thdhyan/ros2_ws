import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from ros_gz_interfaces.srv import SpawnEntity

import os


class TurtlebotNode(Node):
    def __init__(self):
        super().__init__("robot_node")
        self.get_logger().info("TurtleBot3 Node Started")

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

        # Create a client to spawn entities in Gazebo
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Spawn Service...")

        self.get_logger().info("Spawn Service is Ready")

        # Spawn the TurtleBot3 in the simulation
        self.spawn_turtlebot3()

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

    def spawn_turtlebot3(self, name="turtlebot3", x=0.0, y=0.0, z=0.0):
        """Spawn TurtleBot3 in Gazebo"""
        request = SpawnEntity.Request()
        request.name = name

        # Load the TurtleBot3 SDF from the ROS2 package
        sdf_path = os.path.join(
            os.getenv("TURTLEBOT3_MODEL_PATH", "/usr/share/turtlebot3_gazebo/models/turtlebot3_waffle"),
            "model.sdf"
        )
        with open(sdf_path, "r") as sdf_file:
            request.xml = sdf_file.read()

        request.robot_namespace = name
        request.initial_pose = Pose()
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z

        self.get_logger().info(f"Spawning {name} at ({x}, {y}, {z})...")
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"{name} spawned successfully!")
        else:
            self.get_logger().error(f"Failed to spawn {name}: {future.exception()}")


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
