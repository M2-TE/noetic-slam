#!/bin/bash

if [ $NOETICSLAM_DOCKER ]
then
    roslaunch ouster_ros replay.launch\
        bag_file:=/root/repo/bags/${FILENAME}.bag\
        viz:=${RVIZ_OUSTER}
else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && source /root/repo/devel/setup.bash && /root/repo/scripts/ouster-replay.sh"
fi