FROM osrf/ros:humble-desktop-full

# Install ROS2 control and Gazebo Harmonic packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz-sim \
    # ros-humble-turtlebot3-description \
    && rm -rf /var/lib/apt/lists/*



# Set up Gazebo environment variables
ENV GZ_VERSION=harmonic
ENV GZ_RESOURCE_PATH=/usr/share/gz/

# Create a directory for Gazebo cache
RUN mkdir -p /home/root/.gz

# Set the working directory to the ROS2 workspace
WORKDIR /ros2_ws

RUN mkdir -p /ros2_ws/src/turtlebot_node

RUN touch /ros2_ws/src/turtlebot_node/hello
# run the following commands to clone the turtlebot3 packages
# RUN git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git /ros2_ws/src/turtlebot_node/DynamixelSDK
# RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git /ros2_ws/src/turtlebot_node/turtlebot3_msgs
# RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git /ros2_ws/src/turtlebot_node/turtlebot3
# RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git /ros2_ws/src/turtlebot_node/turtlebot3_simulations
# Build the turtlebot_node package
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --packages-select turtlebot_node --symlink-install"

# Copy and set up entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set up entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

