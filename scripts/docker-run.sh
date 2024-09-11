#!/bin/bash
if [ -z $ROS_DISTRO ]; then
    ROS_DISTRO="null"
fi
if [ $ROS_DISTRO = "noetic" ]; then
    catkin config --init --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=/usr/bin/gcc-11 -DCMAKE_CXX_COMPILER=/usr/bin/g++-11
    catkin build
    exit 0
fi

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
docker build -t noeticslam:latest $SCRIPT_DIR/..

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
        --ulimit nofile=1024 \
        --volume $(pwd)/$(dirname "$0")/..:/root/repo/:Z \
        --cap-add=SYS_PTRACE \
        noeticslam:latest
elif [ $1 = "integrated" ]; then
    # with intel integrated gpu
    xhost +local:docker
    docker run -it \
        --rm \
        --name noeticslam \
        --publish 7008:7008/udp \
        --publish 7009:7009/udp \
        --device=/dev/dri:/dev/dri \
        --ulimit nofile=1024 \
        --env DISPLAY=${DISPLAY} \
        --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume $(pwd)/$(dirname "$0")/..:/root/repo/:Z \
        --cap-add=SYS_PTRACE \
        noeticslam:latest
elif [ $1 = "nvidia" ]; then
    # with nvidia gpu
    xhost +local:docker
    docker run -it \
        --rm \
        --name noeticslam \
        --publish 7008:7008/udp \
        --publish 7009:7009/udp \
        --ulimit nofile=1024 \
        --runtime nvidia \
        --gpus all \
        --env DISPLAY=${DISPLAY} \
        --env __NV_PRIME_RENDER_OFFLOAD=1 \
        --env __GLX_VENDOR_LIBRARY_NAME=nvidia \
        --env NVIDIA_VISIBLE_DEVICES=all \
        --env NVIDIA_DRIVER_CAPABILITIES=all \
        --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --volume $(pwd)/$(dirname "$0")/..:/root/repo/:Z \
        --cap-add=SYS_PTRACE \
        noeticslam:latest
elif [ $1 = "amd" ]; then
    # with amd gpu
    # xhost +local:docker
    echo "amd implementation not tested yet (need an amd gpu first)"
fi
