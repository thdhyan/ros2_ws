# ROS2 Turtlebot Simulation System

This workspace contains a ROS2-based simulation system with Gazebo integration, featuring a Turtlebot that responds to published messages.

## Package Structure

- **gazebo_node**: Handles Gazebo simulation and robot spawning
- **talker_node**: Publishes messages on the 'chatter' topic
- **listener_node**: Subscribes to messages on the 'chatter' topic
- **turtlebot_node**: Controls Turtlebot movement and behavior

## Prerequisites

- ROS2 Humble or later
- Gazebo
- colcon build tools
- Python 3.8+

## Building the Workspace

```bash
# Navigate to workspace root
cd /ros2_ws

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Running the System

### Launch Everything
```bash
ros2 launch gazebo_node gazebo_launch.py
```

### Launch Individual Nodes
```bash
# Launch Talker Node
ros2 launch talker_node talker_launch.py

# Launch Listener Node
ros2 launch listener_node listener_launch.py

# Launch Turtlebot Node
ros2 launch turtlebot_node turtlebot_launch.py
```

## Node Information

### Gazebo Node
- Launches Gazebo simulator
- Spawns Turtlebot in the simulation
- Manages simulation environment

### Talker Node
- Publishes messages on 'chatter' topic
- Message frequency: 2Hz
- Message type: std_msgs/String

### Listener Node
- Subscribes to 'chatter' topic
- Logs received messages
- Message type: std_msgs/String

### Turtlebot Node
- Controls Turtlebot movement
- Subscribes to 'chatter' for commands
- Publishes to cmd_vel for movement
- Subscribes to odom for position tracking

## Topics

- /chatter: Communication between talker and listener
- /cmd_vel: Turtlebot velocity commands
- /odom: Turtlebot odometry data

## Development

To add new features or modify existing ones:
1. Make changes to the desired node
2. Rebuild the workspace using `colcon build`
3. Source the workspace again
4. Launch the system

## Troubleshooting

If you encounter issues:
1. Ensure all dependencies are installed
2. Check if Gazebo is properly installed
3. Verify ROS2 environment is properly sourced
4. Check robot spawning issues in Gazebo logs

