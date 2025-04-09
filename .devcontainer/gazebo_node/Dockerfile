FROM osrf/ros:humble-desktop-full

# Add Gazebo repository
RUN apt-get update && apt-get install -y wget lsb-release gnupg curl && \
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo harmonic and ROS-GZ integration packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz \
    ros-humble-ros-ign-bridge \
    # gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Set up Gazebo environment variables
ENV GZ_VERSION=harmonic
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-7/plugins
ENV GZ_RESOURCE_PATH=/usr/share/gz

# Create a non-root user 'ros' and switch to it
# RUN useradd -m ros && \
#     mkdir -p /home/ros/.gz && \
#     chown -R ros:ros /home/ros

# Set the working directory to the ROS2 workspace
WORKDIR /ros2_ws

# copy src/gazebo_node files
COPY src/gazebo_node src/gazebo_node


RUN git clone -b new_gazebo https://github.com/azeey/turtlebot3_simulations.git /ros2_ws/src/turtlebot3_simulations

# In turtlebot3_simulations_ci.repos change ros2-devel to humble
RUN sed -i 's/ros2-devel/humble/g' /ros2_ws/src/turtlebot3_simulations/turtlebot3_simulations_ci.repos
# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && vcs import src < /ros2_ws/src/turtlebot3_simulations/turtlebot3_simulations_ci.repos && colcon build --symlink-install"

# Copy and set up entrypoint
COPY .devcontainer/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set up entrypoint
ENTRYPOINT ["/entrypoint.sh"]
# CMD ["bash"]
