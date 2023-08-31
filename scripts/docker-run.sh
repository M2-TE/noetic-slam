#!/bin/sh
docker build -t noeticslam:latest $(dirname "$0")/..
xhost +local:docker
docker run -it \
    --name noeticslam \
    --rm \
    --privileged \
    --mount type=bind,source="$(pwd)/$(dirname "$0")"/..,target=/root/repo \
    --env DISPLAY=$DISPLAY \
    --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    noeticslam:latest \
    sh scripts/compound-launch.sh
