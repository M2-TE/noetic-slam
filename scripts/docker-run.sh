#!/bin/bash

if [ $NOETICSLAM_DOCKER ]
then
    echo "Use this script from the host, not the container!"
    exit 1
fi

docker build -t noeticslam:latest $(dirname "$0")/..
xhost +local:docker
docker run -it \
    --name noeticslam \
    --rm \
    --privileged \
    --publish 7008:7008/udp \
    --publish 7009:7009/udp \
    --mount type=bind,source="$(pwd)/$(dirname "$0")"/..,target=/root/repo \
    --env DISPLAY=$DISPLAY \
    --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    noeticslam:latest
