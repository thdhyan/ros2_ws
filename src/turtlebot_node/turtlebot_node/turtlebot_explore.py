# Write a python node that explores the world and saves image and laser scan data with its xyz coordinates and does rtabmap-ros to save slam maps to
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDepth

class Turtlebot3_Node(Node):
    def __init__(self):
        super().__init__('turtlebot_node')

        # Create a publisher for the image data
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)

        # Create a publisher for the laser scan data
        self.laser_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)

        # Create a subscriber for the robot's position
        self.position_sub = self.create_subscription(String, '/robot/position', self.position_callback, 10)

        # Create a timer to periodically publish image and laser scan data
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Initialize the position variable
        self.position = None
        # Create a directory to save the data
        self.setup_dirs()
    
    def setup_dirs(self):
        # Create a directory to save the data
        self.data_dir = os.path.join(os.getcwd(), 'data')
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
        # Create a directory to save the images
        self.image_dir = os.path.join(self.data_dir, 'images')
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        # Create a directory to save the laser scan data
        self.laser_dir = os.path.join(self.data_dir, 'laser')
        if not os.path.exists(self.laser_dir):
            os.makedirs(self.laser_dir)
        # Create a directory to save the maps
        self.map_dir = os.path.join(self.data_dir, 'maps')
        if not os.path.exists(self.map_dir):
            os.makedirs(self.map_dir)
            