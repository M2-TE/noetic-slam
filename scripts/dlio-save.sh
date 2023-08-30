#!/bin/sh
if [ $# -eq 0 ]
then
    echo "Missing leaf node size argument"
else
    docker exec -it noeticslam bash -c "\
    . /opt/ros/noetic/setup.bash &&\
    . devel/setup.bash &&\
    rosservice call /robot/dlio_map/save_pcd $1 /root/repo/maps/ &&\
    pcl_pcd2ply /root/repo/maps/dlio_map.pcd /root/repo/maps/dlio_map.ply"
fi