# Extend from base image
FROM maly_woz/base:jazzy

# Install visualization and developer tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    gdb \
    ros-jazzy-rtabmap \
    ros-jazzy-rtabmap-launch \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt* \
    ros-jazzy-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# Install additional packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# Enable GUI apps (X11 forwarding)
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV XAUTHORITY=/tmp/.docker.xauth

CMD ["/bin/bash"]
