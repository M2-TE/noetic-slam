#!/bin/sh
if [ $# -eq 0 ]
then
    echo "Missing path argument (relative to workspace root)"
else
    docker exec -it noeticslam sh -c ". /opt/ros/noetic/setup.sh && rosbag play $1"
fi