#!/bin/sh
if [ $NOETICSLAM_DOCKER ]
then
    sh -c "\
    rosbag play \$BAG_PATH"
else
    docker exec -it noeticslam sh -c "\
    . /opt/ros/noetic/setup.sh &&\
    rosbag play $1"
fi