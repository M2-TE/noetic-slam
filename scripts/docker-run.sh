#!/bin/sh
docker run -d -it --name noeticslam --mount type=bind,source="$(pwd)/..",target=/root/repo noeticslam:latest