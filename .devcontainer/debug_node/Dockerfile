FROM osrf/ros:humble-desktop-full

# Install debugging tools and visualization utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    valgrind \
    ros-humble-rqt* \
    glmark2 \
    x11-apps \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace
WORKDIR /ros2_ws


COPY src/ /ros2_ws/src

# RUN git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git /ros2_ws/src/DynamixelSDK
# RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git /ros2_ws/src/turtlebot3_msgs
# RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git /ros2_ws/src/turtlebot3
# RUN git clone -b new_gazebo https://github.com/azeey/turtlebot3_simulations.git /ros2_ws/src/turtlebot3_simulations

RUN bash -c " source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source /ros2_ws/install/setup.bash "

# Copy and set up entrypoint
COPY .devcontainer/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set up entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

