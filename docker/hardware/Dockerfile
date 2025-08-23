# Extend from base image for ARM64 (Raspberry Pi 4B)
FROM maly_woz/base:jazzy

# Add udev rules (for USB devices, sensors, etc.)
# Copy from your repo into /etc/udev/rules.d/
#COPY src/robot/my_robot_hw_drivers/udev_rules/*.rules /etc/udev/rules.d/

# Source ROS automatically
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
