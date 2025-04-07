FROM osrf/ros:humble-desktop-full

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install development tools and X11/OpenGL dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    gdb \
    git \
    python3-pip \
    python3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    curl \
    vim \
    nano \
    bash-completion \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    mesa-utils \
    mesa-utils-extra \
    x11-apps \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Install NVIDIA container toolkit dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglvnd-dev \
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    && rm -rf /var/lib/apt/lists/*

# Set up environment variables for X11 forwarding and NVIDIA
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

# Setup ROS2 environment for both root and the new user
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/$USERNAME/.bashrc

# Create entry point
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the working directory
WORKDIR /ros2_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
