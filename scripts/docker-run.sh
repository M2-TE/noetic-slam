#!/bin/sh
docker stop noeticslam
docker rm -f noeticslam
docker build -t noeticslam:latest $(dirname "$0")/..
xhost +local:docker
docker run -dit \
    --name noeticslam \
    --privileged \
    --mount type=bind,source="$(pwd)/$(dirname "$0")"/..,target=/root/repo \
    --env DISPLAY=$DISPLAY \
    --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    noeticslam:latest
docker exec -d noeticslam /bin/bash -c '. /opt/ros/noetic/setup.sh && roscore'
docker exec -it noeticslam /bin/bash