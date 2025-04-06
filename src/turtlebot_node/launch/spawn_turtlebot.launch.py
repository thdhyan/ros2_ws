
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node



def generate_launch_description():
    # Configure ROS nodes for launch
    
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    turtlebot3_gazebo_launch = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_gazebo_launch_dir = os.path.join(turtlebot3_gazebo_launch, 'launch') 
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='-2.0')
    
    world = LaunchConfiguration('world', default="empty.sdf")


    # Setup project paths
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )
    
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_launch_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_launch_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )
    # Add Rviz for camera and robot visualization
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_bringup'),'launch','rviz2.launch.py') ,
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    cartographer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'configuration_directory': PathJoinSubstitution([get_package_share_directory('turtlebot3_cartographer'), 'config']),
            'configuration_basename': 'turtlebot3_lds_2d.lua'
        }.items()
    )
    
    return LaunchDescription([
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        rviz_cmd
    ])