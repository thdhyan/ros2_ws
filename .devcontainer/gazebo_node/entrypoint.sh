#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Source workspace setup if it exists
if [ -f "/ros2_ws/install/setup.bash" ]; then
  source "/ros2_ws/install/setup.bash"
fi

# Execute the command passed to docker run
exec "$@"
