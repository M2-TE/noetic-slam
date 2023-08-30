#!/bin/sh
if [ $# -lt 2 ]
then
    echo "Need two arguments: leaf_node_size file_name"
else
    docker exec -it noeticslam bash -c "\
    . /opt/ros/noetic/setup.bash &&\
    . devel/setup.bash &&\
    rosservice call /robot/dlio_map/save_pcd $1 /root/repo/maps/ &&\
    mv /root/repo/maps/dlio_map.pcd /root/repo/maps/$2.pcd &&\
    pcl_pcd2ply /root/repo/maps/$2.pcd /root/repo/maps/$2.ply"
fi