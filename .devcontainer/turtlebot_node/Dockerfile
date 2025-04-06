FROM osrf/ros:jazzy-desktop-full

# Install ROS2 control and Gazebo Harmonic packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz-sim \
    && rm -rf /var/lib/apt/lists/*

# Set up Gazebo environment variables
ENV GZ_VERSION=harmonic
ENV GZ_RESOURCE_PATH=/usr/share/gz/

# Create a directory for Gazebo cache
RUN mkdir -p /home/root/.gz

# Set the working directory to the ROS2 workspace
WORKDIR /ros2_ws

# Build the turtlebot_node package
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /ros2_ws && \
    colcon build --packages-select turtlebot_node --symlink-install"

# Copy and set up entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set up entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

