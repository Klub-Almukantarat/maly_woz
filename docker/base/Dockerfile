# Base ROS 2 Jazzy image
FROM ros:jazzy-ros-core

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install common build tools and colcon
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    curl && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (ignore failure in empty container)
RUN rosdep update || true

# ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    && rm -rf /var/lib/apt/lists/*

# Workspace root (this will be volume-mounted)
WORKDIR /workspace
