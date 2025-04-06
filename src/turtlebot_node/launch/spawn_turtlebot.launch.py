from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        # Launch the office environment in ros_gz_sim
        ExecuteProcess(
            cmd=["ros2", "launch", "ros_gz_sim", "gz_sim.launch.py", "-p", "world:=office"],
            output="screen"
        ),

        # Launch the TurtleBot3 node
        Node(
            package="turtlebot_node",
            executable="turtlebot",
            name="turtlebot_node",
            output="screen"
        ),
    ])
