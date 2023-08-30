#!/bin/sh
docker run -d -it --name noeticslam --mount type=bind,source="$(dirname "$0")/..",target=/root/repo noeticslam:latest