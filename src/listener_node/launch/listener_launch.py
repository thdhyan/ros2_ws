from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='listener_node',
            executable='listener',
            name='listener_node',
            output='screen'
        )
    ])
