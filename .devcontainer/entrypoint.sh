#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Set up colcon defaults if available
if [ -f "/ros2_ws/src/.devcontainer/colcon_defaults.yaml" ]; then
  export COLCON_DEFAULTS_FILE=/ros2_ws/src/.devcontainer/colcon_defaults.yaml
  echo "Using colcon defaults file: $COLCON_DEFAULTS_FILE"
fi

echo "Display is "${DISPLAY}

# ls /ros2_ws/src

# Source the workspace if it exists
if [[ -f "/ros2_ws/install/setup.bash" ]]; then
  # colcon build --symlink-install
  echo "Sourcing the workspace..."
  source "/ros2_ws/install/setup.bash"
fi

chmod 777 -R /ros2_ws
# Set ROS domain ID for network isolation
export ROS_DOMAIN_ID=42

# Set up ROS network configuration
export ROS_LOCALHOST_ONLY=0


# Print ROS environment info
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Using workspace: $([ -f /ros2_ws/install/setup.bash ] && echo 'Yes' || echo 'No')"

chmod 777 -R /ros2_ws/install

# cat /ros2_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/worlds/empty_world.world
# Execute the command passed to the entrypoint
exec "$@"
