#!/bin/sh
docker build -t noeticslam:latest $(dirname "$0")/..
xhost +local:docker
docker run -dit \
    --name noeticslam \
    --rm \
    --privileged \
    --mount type=bind,source="$(pwd)/$(dirname "$0")"/..,target=/root/repo \
    --env DISPLAY=$DISPLAY \
    --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    noeticslam:latest
# start roscore
docker exec -d noeticslam /bin/sh -c '. /opt/ros/noetic/setup.sh && roscore'
# enter container with bash
docker exec -it noeticslam /bin/bash
docker stop noeticslam