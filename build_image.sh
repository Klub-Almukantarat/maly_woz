#!/usr/bin/env bash
docker build --build-arg="user_id=1000" --rm -t almu_ros2_base docker/almu_ros2_base
docker build --rm -t almu_rover docker
