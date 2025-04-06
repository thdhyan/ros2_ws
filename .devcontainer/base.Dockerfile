FROM osrf/ros:humble-desktop-full

# Install basic development tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    gdb \
    gdbserver \
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
    htop \
    tmux \
    lsb-release \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# Install X11 and GL dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglvnd-dev \
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    libgles2-mesa-dev \
    libglx-dev \
    libegl1 \
    libxext6 \
    libx11-6 \
    mesa-utils \
    x11-utils \
    && rm -rf /var/lib/apt/lists/*

# Set up environment for GPU acceleration
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,display,utility,compute
ENV QT_X11_NO_MITSHM=1

# Create non-root user with sudo access
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set up ROS2 environment
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> /home/$USERNAME/.bashrc

# Define default workdir
WORKDIR /ros2_ws

# Copy the entrypoint script
COPY .devcontainer/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

