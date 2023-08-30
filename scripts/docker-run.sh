#!/bin/sh
docker rm --force noeticslam
docker run -d -it --name noeticslam --mount type=bind,source="$(pwd)/$(dirname "$0")"/..,target=/root/repo noeticslam:latest