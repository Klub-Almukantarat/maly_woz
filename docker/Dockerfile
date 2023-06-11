FROM osrf/ros:humble-desktop-full

ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL /bin/bash

USER $USERNAME

# install required packages
RUN sudo apt-get update && sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# RUN pip install \
#     scipy

WORKDIR /home/developer/rover_ws
CMD ["/bin/bash"]
