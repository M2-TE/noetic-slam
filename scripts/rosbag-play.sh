#!/bin/bash

if [ $NOETICSLAM_DOCKER ]
then
    rosbag play $BAG_PATH
else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && /root/repo/scripts/rosbag-play.sh"
fi