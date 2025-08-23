#!/usr/bin/env bash

echo "Building OpenGL base images..."
(cd docker/opengl ; ./build.sh)
echo "Done"

echo "Building base image..."
docker buildx build --rm -t maly_woz/base:jazzy docker/base && echo "Done"
echo "Building hardware image..."
docker buildx build --rm -t maly_woz/hardware:jazzy docker/hardware && echo "Done"
echo "Building software image..."
docker buildx build --rm -t maly_woz/software:jazzy docker/software && echo "Done"
echo "Building sim image..."
docker buildx build --rm -t maly_woz/sim:jazzy docker/sim && echo "Done"
#docker build --build-arg="user_id=1000" --rm -t almu_ros2_base docker/almu_ros2_base
#docker build --rm -t almu_rover docker
