import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('gazebo_node'), 'launch')
    pkg_gazebo_node = get_package_share_directory('gazebo_node')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Define launch configurations
    world_name = LaunchConfiguration('world_name', default='empty_world.world')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    # Declare the launch arguments
    
    # Append the worlds directory into the GZ_SIM_RESOURCE_PATH environment variable
    set_env_vars_resources = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=":".join([os.path.join(pkg_gazebo_node, 'worlds'), os.path.join(pkg_gazebo_node, 'models')]),
    )
    
    def launch_gz_sim(context):
        # Get the world name from the launch configuration
        if 'world_name' not in context.launch_configurations.keys():
            world_name = 'empty_world.world'
        else:
            world_name = context.launch_configurations['world_name']
        # Get the world name from the launch configuration
        world_name_extact = os.path.join(pkg_gazebo_node, 'worlds', world_name)
        # Return the world name as a string
        return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [f' -r -s -v4 {world_name_extact}']
        }.items()
        )  ]  
    
    gzserver_cmd= OpaqueFunction(function=launch_gz_sim)
    
    # Use an OpaqueFunction to delay world resolution until launch
    
    # Include the gzclient command (for GUI)
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-g ']}.items()
    )
    
    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    
    return ld