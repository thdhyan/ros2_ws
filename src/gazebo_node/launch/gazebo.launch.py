from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the Gazebo launch directory
    pkg_gazebo_node = get_package_share_directory("gazebo_node")
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_node, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"gui": "true"}.items()
    )
    
    # Launch nodes
    gazebo_node = Node(
        package="gazebo_node",
        executable="gazebo_node",
        name="gazebo_node",
        output="screen"
    )
    
    talker_node = Node(
        package="talker_node",
        executable="talker",
        name="talker_node",
        output="screen"
    )
    
    listener_node = Node(
        package="listener_node",
        executable="listener",
        name="listener_node",
        output="screen"
    )
    
    turtlebot_node = Node(
        package="turtlebot_node",
        executable="turtlebot",
        name="turtlebot_node",
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        gazebo_node,
        talker_node,
        listener_node,
        turtlebot_node
    ])
