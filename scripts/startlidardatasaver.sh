#!/bin/bash

# Save data from scanner in file structure

if [ -z $ROS_DISTRO ]; then
    ROS_DISTRO="null"
fi
if [ $ROS_DISTRO = "noetic" ]; then
    SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
    source /opt/ros/noetic/setup.bash
    source ${SCRIPT_DIR}/../devel/setup.bash
    rosrun lidardatasaver lidardatasaver_node
else
    docker exec -it noeticslam bash -c "/root/repo/scripts/startlidardatasaver.sh"
fi