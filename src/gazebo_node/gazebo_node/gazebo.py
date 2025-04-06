import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity
import os

class GazeboNode(Node):
    def __init__(self):
        super().__init__("gazebo_node")
        self.get_logger().info("Gazebo Node Started")
        
        # Create a client to spawn entities in Gazebo
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for Spawn Service...")
            
        self.get_logger().info("Spawn Service is Ready")

    def spawn_turtlebot(self, name="turtlebot", x=0.0, y=0.0, z=0.0):
        """Spawn TurtleBot in Gazebo"""
        request = SpawnEntity.Request()
        request.name = name
        request.xml = """<?xml version="1.0"?>
        <robot name="turtlebot3_waffle">
          <!-- TurtleBot3 URDF -->
        </robot>"""
        request.robot_namespace = ""
        request.initial_pose = Pose()
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z
        
        future = self.spawn_client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    node = GazeboNode()
    
    try:
        # Spawn TurtleBot
        future = node.spawn_turtlebot()
        rclpy.spin_until_future_complete(node, future)
        
        if future.result() is not None:
            node.get_logger().info("TurtleBot spawned successfully")
        else:
            node.get_logger().error("Failed to spawn TurtleBot")
        
        # Keep the node running
        rclpy.spin(node)
        
    except Exception as e:
        node.get_logger().error(f"Error occurred: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
