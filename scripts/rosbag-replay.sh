#!/bin/bash

if [ $ROS_DISTRO = "noetic" ]; then
    if [ -z "$1" ]; then
        echo "usage: scripts/rosbag-replay [bags/*.bag]"
        exit -1
    fi
    SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
    rosbag play ${SCRIPT_DIR}/../$1
else
    docker exec -it noeticslam bash -c "source /opt/ros/noetic/setup.bash && /root/repo/scripts/rosbag-replay.sh $1"
fi