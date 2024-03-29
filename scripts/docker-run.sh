#!/bin/bash

if [ $NOETICSLAM_DOCKER ]; then
    echo "Use this script from the host, not the container!"
    exit 1
fi

docker build -t noeticslam:latest $(dirname "$0")/..

selected_mode="integrated"
if [ -n "$1" ]; then
    selected_mode=$1
fi

if [ $selected_mode = "integrated" ]; then
    # with intel integrated gpu
    xhost +local:docker
    docker run -it \
        --rm \
        --name noeticslam \
        --publish 7008:7008/udp \
        --publish 7009:7009/udp \
        --mount type=bind,source="$(pwd)/$(dirname "$0")"/..,target=/root/repo \
        --device=/dev/dri:/dev/dri \
        --ulimit nofile=1024 \
        --env DISPLAY=${DISPLAY} \
        --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        noeticslam:latest
elif [ $selected_mode = "nvidia" ]; then
    # with nvidia gpu
    xhost +local:docker
    docker run -it \
        --rm \
        --name noeticslam \
        --publish 7008:7008/udp \
        --publish 7009:7009/udp \
        --mount type=bind,source="$(pwd)/$(dirname "$0")"/..,target=/root/repo \
        --ulimit nofile=1024 \
        --gpus all \
        --env DISPLAY=${DISPLAY} \
        --env __NV_PRIME_RENDER_OFFLOAD=1 \
        --env __GLX_VENDOR_LIBRARY_NAME=nvidia \
        --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --runtime nvidia \
        noeticslam:latest
elif [ $selected_mode = "amd" ]; then
    # with amd gpu
    # xhost +local:docker
    echo "amd implementation not tested yet (need an amd gpu first)"
fi
