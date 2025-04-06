FROM osrf/ros:humble-desktop-full

# Set the working directory to the ROS2 workspace
WORKDIR /ros2_ws

# Build the listener_node package
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --packages-select listener_node --symlink-install"

# Copy and set up entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set up entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

