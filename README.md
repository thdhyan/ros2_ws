# ROS2 TurtleBot Workspace

A containerized ROS2 humble workspace for TurtleBot4 simulation in Gazebo Harmonic, including message publishing/subscription examples.

![TurtleBot4 in Gazebo](https://raw.githubusercontent.com/turtlebot/turtlebot4_tutorials/humble/turtlebot4_simulator/images/tb4_gazebo.png)

## Project Overview

This workspace provides a complete development environment for working with TurtleBot4 in Gazebo Harmonic using ROS2 humble. The architecture includes:

- **Containerized Environment**: All dependencies are containerized using Docker to ensure consistent development and deployment
- **Multi-Node Architecture**: Separate nodes for different functionalities (debugging, communication, simulation, robot control)
- **GPU Acceleration**: NVIDIA GPU support for Gazebo simulation
- **ROS2 Control**: Full support for ROS2 control framework with differential drive controllers
- **X11 Integration**: GUI application support for visualization tools

### System Architecture

```
┌─────────────────┐      ┌─────────────────┐      ┌─────────────────┐
│                 │      │                 │      │                 │
│   debug_node    │      │  talker_node    │      │ listener_node   │
│                 │      │                 │      │                 │
└────────┬────────┘      └────────┬────────┘      └────────┬────────┘
         │                        │                         │
         │                        │                         │
         │                        ▼                         │
         │               ┌─────────────────┐                │
         │               │                 │                │
         └───────────────►    ROS2 DDS     ◄────────────────┘
                         │                 │
                         └────────┬────────┘
                                  │
                                  │
         ┌─────────────────┐      │      ┌─────────────────┐
         │                 │      │      │                 │
         │  gazebo_node    ◄──────┴──────►  turtlebot_node │
         │                 │             │                 │
         └─────────────────┘             └─────────────────┘
```

## Prerequisites

- Ubuntu 20.04 or newer
- Docker Engine 20.10+
- Docker Compose v2
- NVIDIA GPU with appropriate drivers (for simulation)
- X11 for GUI applications

### Hardware Requirements

- 4+ CPU cores recommended
- 8GB+ RAM recommended
- NVIDIA GPU with 4GB+ VRAM recommended for simulation
- 20GB+ free disk space

## Network Configuration

This workspace uses ROS2's DDS middleware for node communication across containers.

### ROS_DOMAIN_ID

All containers use `ROS_DOMAIN_ID=42` by default to isolate ROS2 communication from other ROS2 systems on the network. To change this:

```bash
# Edit in docker-compose.yml or override at runtime
docker compose run -e ROS_DOMAIN_ID=99 debug_node
```

### Container Network Configuration

The containers use host network mode for simplicity and performance:

```yaml
# From docker-compose.yml
network_mode: host
```

This allows:
- Direct communication between containers using ROS2 DDS
- X11 forwarding without additional configuration
- Simplified port mapping for services

### Cross-Container Communication

- Topics can be published/subscribed across all containers
- Services and actions work seamlessly across container boundaries
- Parameters are synchronized within the same ROS domain

### Monitoring Node Communication

To view active nodes and their topics:

```bash
# List all active nodes
docker exec ros2_debug_node ros2 node list

# View topics for a specific node
docker exec ros2_debug_node ros2 node info /talker

# View all active topics
docker exec ros2_debug_node ros2 topic list

# Monitor messages on a topic
docker exec ros2_debug_node ros2 topic echo /chatter
```

### Network Troubleshooting

If nodes cannot discover each other:

1. Ensure all containers use the same ROS_DOMAIN_ID
2. Verify network_mode is set correctly in docker-compose.yml
3. Check if any firewall is blocking multicast traffic (DDS discovery uses multicast)

```bash
# Test node discovery between containers
docker exec ros2_debug_node ros2 topic pub /test std_msgs/String "data: test" --once
docker exec ros2_listener_node ros2 topic echo /test
```

## Setup Instructions

1. Clone this repository:

```bash
git clone https://github.com/yourusername/ros2_ws.git
cd ros2_ws
```

2. Ensure Docker has access to the X server:

```bash
xhost +local:docker
```

3. Configure GPU access for Docker (if using NVIDIA GPU):

```bash
# Install NVIDIA Container Toolkit
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

## Build Instructions

Build all the containers using docker-compose:

```bash
# Build all services
docker compose build

# Build a specific service
docker compose build turtlebot_node
```

## Running the Workspace

### Running All Nodes

To start all nodes in the correct order:

```bash
docker compose up
```

For graceful shutdown:

```bash
# Gracefully shut down all containers
docker compose down

# Stop specific service with proper signal handling
docker compose stop turtlebot_node
```

This will start the nodes in dependency order:
1. debug_node
2. talker_node
3. listener_node
4. gazebo_node
5. turtlebot_node

### Running Individual Nodes

To run a specific node and its dependencies:

```bash
# Run just the TurtleBot node (will also start Gazebo)
docker compose up turtlebot_node

# Run just the communication nodes
docker compose up listener_node
```

To run a node in interactive mode:

```bash
docker compose run debug_node bash
```

## Node Descriptions

### debug_node

Purpose: Environment verification and debugging
- Verifies ROS2 environment is correctly configured
- Provides a sandbox for testing and debugging
- Contains debugging and visualization tools

### talker_node

Purpose: Demonstrates message publishing
- Publishes `std_msgs/String` messages on the `/chatter` topic
- Configured to send 10 messages and exit by default
- Demonstrates proper ROS2 publisher setup

### listener_node

Purpose: Demonstrates message subscription
- Subscribes to the `/chatter` topic
- Processes incoming messages
- Demonstrates proper ROS2 subscriber implementation

### gazebo_node

Purpose: Simulation environment
- Runs Gazebo Harmonic simulator
- Provides physics simulation and visualization
- Creates the world environment for the robot

### turtlebot_node

Purpose: Robot spawning and control
- Spawns the TurtleBot4 model in Gazebo
- Provides controllers for robot movement
- Integrates ROS2 Control for actuation
- Connects sensors to ROS2 topics

## Development Workflow

### Creating New Packages

```bash
# Enter the development container
docker compose run debug_node bash

# Create a new package
cd /ros2_ws/src
ros2 pkg create my_new_package --build-type ament_cmake --dependencies rclcpp
```

### Building Packages

```bash
# Build the entire workspace
cd /ros2_ws
colcon build 

# Build specific packages
colcon build --packages-select turtlebot_node
```

### Running Tests

```bash
# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select turtlebot_node
```

### Workflow Examples

#### Developing a New Node

1. Create the package and node code in the `src` directory
2. Build the package: `colcon build --packages-select my_package`
3. Source the workspace: `source /ros2_ws/install/setup.bash`
4. Run your node: `ros2 run my_package my_node`

#### Modifying TurtleBot Configuration

1. Edit files in `/ros2_ws/src/turtlebot_node/config`
2. Rebuild the package: `colcon build --packages-select turtlebot_node`
3. Restart the turtlebot node: `docker compose restart turtlebot_node`

## Troubleshooting

### Common Issues

#### X11 Display Issues

**Symptom**: GUI applications fail to open with "Cannot open display" errors.

**Solution**:
```bash
# Allow local connections to X server
xhost +local:docker

# Verify DISPLAY environment variable is correctly set
echo $DISPLAY
```

#### Permission Denied Errors

**Symptom**: "Permission denied" when accessing devices or files.

**Solution**: 
```bash
# If accessing /dev/dri
sudo chmod a+rw /dev/dri/*

# If file permission issues
sudo chown -R $USER:$USER .
```

#### Gazebo Crashes or Hangs

**Symptom**: Gazebo doesn't start properly or crashes during startup.

**Solution**:
```bash
# Check NVIDIA drivers
nvidia-smi

# Clean up any stale Gazebo processes
pkill -f gazebo

# Try increasing GPU memory in docker-compose.yml
```

### Logging and Debugging

To see logs from a specific container:

```bash
docker compose logs turtlebot_node

# Follow logs in real-time
docker compose logs -f turtlebot_node
```

For more detailed debugging:

```bash
# Run with ROS_LOG_LEVEL set to debug
docker compose run -e ROS_LOG_LEVEL=DEBUG turtlebot_node
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgements

- [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)
- [TurtleBot4 Project](https://turtlebot.github.io/turtlebot4-user-manual/)
- [Gazebo Simulation](https://gazebosim.org/home)
