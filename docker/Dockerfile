FROM almu_ros2_base

USER root

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash
RUN mkdir -p /workspace/ros_ws && \
    chown -R $USERNAME:$USERNAME /workspace && \
    chown -R $USERNAME:$USERNAME /home/developer

USER $USERNAME

# install required packages
RUN sudo apt-get update && sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rtabmap-ros \
    ros-humble-image-pipeline

# RUN pip install \
#     scipy

WORKDIR /workspace/ros_ws
CMD ["/bin/bash"]
