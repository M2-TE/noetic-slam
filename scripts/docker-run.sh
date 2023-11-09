#!/bin/bash

if [ $NOETICSLAM_DOCKER ]; then
    echo "Use this script from the host, not the container!"
    exit 1
fi

docker build -t noeticslam:latest $(dirname "$0")/..
xhost +local:docker

# with nvidia gpu
docker run -it \
    --name noeticslam \
    --rm \
    --privileged \
    --publish 7008:7008/udp \
    --publish 7009:7009/udp \
    --mount type=bind,source="$(pwd)/$(dirname "$0")"/..,target=/root/repo \
    --gpus all \
    --env DISPLAY=$DISPLAY \
    --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --runtime nvidia \
    noeticslam:latest
docker_status=$?

# dont mount gpus when none can be found
if [ $docker_status -ne 0 ]; then
    echo "GPU passthrough failed, falling back to default"
    # with intel integrated gpu
    docker run -it \
        --name noeticslam \
        --rm \
        --privileged \
        --publish 7008:7008/udp \
        --publish 7009:7009/udp \
        --mount type=bind,source="$(pwd)/$(dirname "$0")"/..,target=/root/repo \
        --device=/dev/dri:/dev/dri \
        --env DISPLAY=$DISPLAY \
        --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        noeticslam:latest
fi
