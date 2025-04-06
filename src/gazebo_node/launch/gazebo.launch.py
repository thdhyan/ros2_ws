from launch import LaunchDescription
from launch.actions import ExecuteProcess

# Launch ros_gz_sim in empty world
def generate_launch_description():
    
    ld = LaunchDescription()
    print("Starting Gazebo Node...")
    
    # Add The gz_sim Process
    # Add Turtlebot Node sdf in the world
    
    ld.add_action(
        ExecuteProcess(
            cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py', '-p', 'world:=empty'],
            output='screen'
        )
    )
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py', '-p', 'world:=empty'],
            output='screen'
        ),
    ])

