# Base ROS 2 Jazzy image for ARM64 (Raspberry Pi 4B)
FROM arm64v8/ros:jazzy-ros-core

ENV DEBIAN_FRONTEND=noninteractive

# Install build essentials and common ROS tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget curl \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (tolerate failures if no internet in container)
RUN rosdep update || true

# Add udev rules (for USB devices, sensors, etc.)
# Copy from your repo into /etc/udev/rules.d/
COPY src/robot/my_robot_hw_drivers/udev_rules/*.rules /etc/udev/rules.d/

# Source ROS automatically
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Default workspace (will be mounted or copied at build)
WORKDIR /workspace

CMD ["/bin/bash"]
