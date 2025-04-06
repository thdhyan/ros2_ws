from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    print("Starting Turtlebot Node...")
    return LaunchDescription([
        Node(
            package='turtlebot_node',
            executable='turtlebot',
            name='turtlebot_node',
            output='screen'
        )
    ])
