#!/bin/bash

if [ $ROS_DISTRO = "noetic" ]; then
    if [ -z "$1" ]; then
        echo "usage: scripts/ouster-replay [bags/*.bag]"
        exit -1
    fi
    roslaunch ouster_ros replay.launch \
        bag_file:=/root/repo/$1 \
        viz:=${RVIZ_OUSTER}
else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && source /root/repo/devel/setup.bash && /root/repo/scripts/ouster-replay.sh $1"
fi