#!/bin/bash

if [ $NOETICSLAM_DOCKER ]
then
    rosbag play /root/repo/bags/${FILENAME}_generic.bag
else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && /root/repo/scripts/rosbag-replay.sh"
fi