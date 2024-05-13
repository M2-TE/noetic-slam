#!/bin/bash

if [ $ROS_DISTRO = "noetic" ]; then
    echo "Use this script from the host, not the container!"
    exit 1
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
echo $SCRIPT_DIR
docker build -t noeticslam:latest $(SCRIPT_DIR)/..

if [ -z "$1" ]; then
    echo "usage: scripts/docker-run [none, integrated, nivida, amd]"
    exit -1
fi

if [ $1 = "none" ]; then
    docker run -it \
        --rm \
        --name noeticslam \
        --publish 7008:7008/udp \
        --publish 7009:7009/udp \
        --volume $(pwd)/$(dirname "$0")/..:/root/repo/:Z \
        --ulimit nofile=1024 \
        noeticslam:latest
elif [ $1 = "integrated" ]; then
    # with intel integrated gpu
    xhost +local:docker
    docker run -it \
        --rm \
        --name noeticslam \
        --publish 7008:7008/udp \
        --publish 7009:7009/udp \
        --volume $(pwd)/$(dirname "$0")/..:/root/repo/:Z \
        --device=/dev/dri:/dev/dri \
        --ulimit nofile=1024 \
        --env DISPLAY=${DISPLAY} \
        --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        noeticslam:latest
elif [ $1 = "nvidia" ]; then
    # with nvidia gpu
    xhost +local:docker
    docker run -it \
        --rm \
        --name noeticslam \
        --publish 7008:7008/udp \
        --publish 7009:7009/udp \
        --volume $(pwd)/$(dirname "$0")/..:/root/repo/:Z \
        --ulimit nofile=1024 \
        --gpus all \
        --env DISPLAY=${DISPLAY} \
        --env __NV_PRIME_RENDER_OFFLOAD=1 \
        --env __GLX_VENDOR_LIBRARY_NAME=nvidia \
        --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --runtime nvidia \
        noeticslam:latest
elif [ $1 = "amd" ]; then
    # with amd gpu
    # xhost +local:docker
    echo "amd implementation not tested yet (need an amd gpu first)"
fi
