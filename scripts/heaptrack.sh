#!/bin/bash
if [ -z $ROS_DISTRO ]; then
    ROS_DISTRO="null"
fi
if [ $ROS_DISTRO = "noetic" ]; then
    heaptrack --pid $(pidof tsdf_map_node)
else
    docker exec -it noeticslam bash -c "/root/repo/scripts/heaptrack.sh"
fi