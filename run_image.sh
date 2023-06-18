#!/usr/bin/env bash
#
# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

#IMG=$(basename $1)
#IMG=osrf/mbzirc:mbzirc_sim_latest
IMG=almu_rover

ARGS=("$@")

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]
    then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

DOCKER_OPTS=""

GPU_VENDOR=`lspci | grep ' 3D ' | cut -d" " -f 4`
if [ -z $GPU_VENDOR ]
then
    GPU_VENDOR=`lspci | grep ' VGA ' | cut -d" " -f 5`
fi

if [ $GPU_VENDOR = "Intel" ]
then
    DOCKER_OPTS="$DOCKER_OPTS -v /dev/dri:/dev/dri --device /dev/dri"
else
    DOCKER_OPTS="$DOCKER_OPTS --gpus all"
fi

# Share your vim settings.
VIMRC=~/.vimrc
if [ -f $VIMRC ]
then
  DOCKER_OPTS="$DOCKER_OPTS -v $VIMRC:/home/developer/.vimrc:ro"
fi

# Prevent executing "docker run" when xauth failed.
if [ ! -f $XAUTH ]
then
  echo "[$XAUTH] was not properly created. Exiting..."
  exit 1
fi

# Mount extra volumes if needed.
# E.g.:
# -v "/opt/sublime_text:/opt/sublime_text" \

# Developer note: If you are running docker in cloudsim then make sure to add
# -e IGN_PARTITION=subt to the following command.
docker run -it \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -e "TERM=xterm-256color" \
  -v "$XAUTH:$XAUTH" \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "/dev/input:/dev/input" \
  -v "$SCRIPTPATH/docker/bash_history:/home/developer/.bash_history" \
  -v "$SCRIPTPATH/docker/.bashrc:/home/developer/.bashrc" \
  -v "$SCRIPTPATH/docker/rviz:/home/developer/.rviz2" \
  -v "$SCRIPTPATH:/home/developer/rover_ws" \
  -v "$SCRIPTPATH/docker/ignition:/home/developer/.ignition" \
  --network host \
  --rm \
  --privileged \
  --security-opt seccomp=unconfined \
  --ipc=host \
  --name="rover" \
  $DOCKER_OPTS \
  $IMG \
  /bin/bash
