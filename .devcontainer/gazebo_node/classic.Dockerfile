FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/
RUN mkdir -p /ros2_ws/src/
COPY src/gazebo_node/ /ros2_ws/src/gazebo_node

# Build the gazebo_node package
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install"

# Copy and set up entrypoint
COPY .devcontainer/gazebo_node/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
