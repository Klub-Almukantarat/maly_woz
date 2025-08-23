# GPU-friendly base (Ubuntu 24.04)
FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive

# Essentials + repo setup helpers
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# --- ROS 2 Jazzy apt repo ---
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# --- Gazebo (Harmonic) apt repo ---
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# Install ROS tools + Gazebo Harmonic + ROS-GZ bridge + GUI tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Gazebo Harmonic (new 'gz' stack)
    gazebo-harmonic \
    # ROS ↔︎ Gazebo integration (bridge, sim wrappers, msgs)
    ros-jazzy-ros-gz \
    # Useful dev/GUI tools
    ros-jazzy-rviz2 \
    ros-jazzy-rqt* \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential cmake git gdb \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (ok if it no-ops in CI)
RUN rosdep update || true

# Environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

# X11 GUI env (forwarded from host)
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV XAUTHORITY=/tmp/.docker.xauth

# NVIDIA runtime hints (compose can also set these)
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

WORKDIR /workspace
CMD ["/bin/bash"]
