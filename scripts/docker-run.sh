#!/bin/sh
docker rm --force noeticslam
docker build -t noeticslam:latest $(dirname "$0")/..
docker run -d --name noeticslam --mount type=bind,source="$(pwd)/$(dirname "$0")"/..,target=/root/repo noeticslam:latest
docker exec -it noeticslam /bin/bash