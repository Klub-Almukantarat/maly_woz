# Extend from base image for ARM64 (Raspberry Pi 4B)
FROM maly_woz/base:jazzy

# Add udev rules (for USB devices, sensors, etc.)
# Copy from your repo into /etc/udev/rules.d/
#COPY src/robot/my_robot_hw_drivers/udev_rules/*.rules /etc/udev/rules.d/

# RPi packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rpi.gpio \
    ros-jazzy-camera-ros \
    && rm -rf /var/lib/apt/lists/*

# libcamera
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip git python3-jinja2 \
    libboost-dev \
    libgnutls28-dev openssl libtiff-dev pybind11-dev \
    qtbase5-dev libqt5core5a libqt5widgets5 \
    meson cmake \
    python3-yaml python3-ply \
    libglib2.0-dev libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# Source ROS automatically
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
