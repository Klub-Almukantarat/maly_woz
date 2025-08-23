# NVIDIA OpenGL runtime (Ubuntu 24.04 base)
FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu24.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Setup ROS 2 Jazzy sources
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# Install ROS Jazzy Desktop (includes Gazebo Harmonic + rviz2, rqt, demos)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    cmake \
    git \
    gdb \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep update || true

# Environment setup
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Enable GUI apps (X11 forwarding)
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV XAUTHORITY=/tmp/.docker.xauth

# Default workspace
WORKDIR /workspace

CMD ["/bin/bash"]
