FROM osrf/ros:humble-desktop-full
# Install ROS2 nav2 packages and pip and 
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    python3-pip\
    && rm -rf /var/lib/apt/lists/*

# USER ros
# Pip install ULtralytics
# RUN pip3 install --upgrade pip

# RUN pip3 install "ultralytics==8.3.91" opencv-python matplotlib 'numpy<2'  pandas scikit-learn scikit-image seaborn tqdm

# change work dir and copy explore_nodex into src
WORKDIR /ros2_ws/
RUN mkdir -p /ros2_ws/src/

RUN git clone 
COPY src/explore_node/ /ros2_ws/src/explore_node

# Build the explore_node package
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install"

# Copy and set up entrypoint
COPY .devcontainer/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
# Set up entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
