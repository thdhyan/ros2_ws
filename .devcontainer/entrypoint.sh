#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# build the wokrspace if it doesn't exist
if [ ! -f "/ros2_ws/install/setup.bash" ]; then
  echo "Building the workspace..."
  mkdir -p /ros2_ws/src
  cd /ros2_ws
  colcon build --symlink-install
else
  echo "Workspace already built."
fi

# Source the workspace if it exists
if [ -f "/ros2_ws/install/setup.bash" ]; then
  colcon build --symlink-install
  echo "Sourcing the workspace..."
  source "/ros2_ws/install/setup.bash"
fi

# Set ROS domain ID for network isolation
export ROS_DOMAIN_ID=42

# Set up ROS network configuration
export ROS_LOCALHOST_ONLY=0

# Set up colcon defaults if available
if [ -f "/ros2_ws/src/.devcontainer/colcon_defaults.yaml" ]; then
  export COLCON_DEFAULTS_FILE=/ros2_ws/src/.devcontainer/colcon_defaults.yaml
fi

# Print ROS environment info
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Using workspace: $([ -f /ros2_ws/install/setup.bash ] && echo 'Yes' || echo 'No')"

# Execute the command passed to the entrypoint
exec "$@"
