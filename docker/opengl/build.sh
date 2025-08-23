#!/bin/bash

export IMAGE_NAME="nvidia/opengl"
export LIBGLVND_VERSION="1.2"

export OS="ubuntu"
export OS_VERSION="24.04"
export FROM="${OS}:${OS_VERSION}"

# Example for building unsupported debian images:
# export FROM="debian:10"
# export OS="debian"
# export OS_VERSION="10"

cp NGC-DL-CONTAINER-LICENSE base/

echo -e ">>> Building BASE Image\n"

docker buildx build -t "${IMAGE_NAME}:base-${OS}${OS_VERSION}" --build-arg "from=${FROM}" "base/"

echo -e "\n>>> Building RUNTIME Image\n"

docker buildx build -t "${IMAGE_NAME}:${LIBGLVND_VERSION}-glvnd-runtime-${OS}${OS_VERSION}" \
             --build-arg "from=${IMAGE_NAME}:base-${OS}${OS_VERSION}" \
             --build-arg "LIBGLVND_VERSION=${LIBGLVND_VERSION}" \
             "glvnd/runtime"

echo -e "\n>>> Building DEVEL Image\n"

docker buildx build -t "${IMAGE_NAME}:${LIBGLVND_VERSION}-glvnd-devel-${OS}${OS_VERSION}" \
             --build-arg "from=${IMAGE_NAME}:${LIBGLVND_VERSION}-runtime-${OS}${OS_VERSION}" \
             "glvnd/devel"
