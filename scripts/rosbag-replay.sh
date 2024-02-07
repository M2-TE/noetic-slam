#!/bin/bash

if [ $NOETICSLAM_DOCKER ]
then
    if [ -z "$1" ]; then
        echo "error: give path to bag relative to repo root as first param"
        exit -1
    fi
    rosbag play /root/repo/bags/$1
else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && /root/repo/scripts/rosbag-replay.sh $1"
fi