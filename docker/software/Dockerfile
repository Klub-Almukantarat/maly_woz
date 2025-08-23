# Extend from base image
FROM my_robot_base:jazzy

# Install visualization and developer tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt* \
    ros-jazzy-tf2-tools \
    ros-jazzy-diagnostic-tools \
    gdb \
    && rm -rf /var/lib/apt/lists/*

# Enable GUI apps (X11 forwarding)
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV XAUTHORITY=/tmp/.docker.xauth

# Default workspace
WORKDIR /workspace

CMD ["/bin/bash"]
