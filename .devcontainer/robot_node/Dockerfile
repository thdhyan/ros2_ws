FROM osrf/ros:humble-desktop-full

# Install ROS2 nav2 and Gazebo Classic packages along with pip
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-ros-gz \
    ros-humble-ros-ign-bridge \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create a directory for Gazebo cache (if needed)
RUN mkdir -p /home/root/.gz

# Set the working directory to the ROS2 workspace
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src/

# Copy your robot_node package source and clone required repositories
COPY src/robot_nodex/ /ros2_ws/src/robot_node
RUN git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git /ros2_ws/src/DynamixelSDK
RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git /ros2_ws/src/turtlebot3_msgs
RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git /ros2_ws/src/turtlebot3
RUN git clone -b new_gazebo https://github.com/azeey/turtlebot3_simulations.git /ros2_ws/src/turtlebot3_simulations

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --symlink-install"

# Copy and set up entrypoint
COPY .devcontainer/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

