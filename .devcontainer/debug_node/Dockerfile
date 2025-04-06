FROM osrf/ros:humble-desktop-full

# Install debugging tools and visualization utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    valgrind \
    ros-humble-rqt* \
    glmark2 \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace
WORKDIR /ros2_ws

# Build the debug_node package if it exists
RUN bash -c "if [ -d /ros2_ws/src/debug_node ]; then \
    source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --packages-select debug_node --symlink-install; \
    fi"

# Copy and set up entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set up entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

